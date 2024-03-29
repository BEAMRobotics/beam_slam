/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2019, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */
#include <bs_optimizers/fixed_lag_smoother.h>

#include <bs_common/imu_state.h>
#include <bs_constraints/inertial/absolute_imu_state_3d_stamped_constraint.h>
#include <bs_parameters/parameter_base.h>
#include <fuse_constraints/marginalize_variables.h>
#include <fuse_core/graph.h>
#include <fuse_core/transaction.h>
#include <fuse_core/uuid.h>
#include <fuse_optimizers/optimizer.h>
#include <ros/ros.h>

#include <algorithm>
#include <iterator>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {
/**
 * @brief Delete an element from the vector using a reverse iterator
 *
 * @param[in] container The contain to delete from
 * @param[in] position  A reverse iterator that access the element to be erased
 * @return A reverse iterator pointing to the element after the erased element
 */
template <typename T>
typename std::vector<T>::reverse_iterator
    erase(std::vector<T>& container,
          typename std::vector<T>::reverse_iterator position) {
  // Reverse iterators are weird
  // https://stackoverflow.com/questions/1830158/how-to-call-erase-with-a-reverse-iterator
  // Basically a reverse iterator access the element one place before the
  // element it points at. E.g. The reverse iterator rbegin points at end, but
  // accesses end-1. When you delete something, you need to increment the
  // reverse iterator first, then convert it to a standard iterator for the
  // erase operation.
  std::advance(position, 1);
  container.erase(position.base());
  return position;
}
} // namespace

namespace bs_optimizers {

FixedLagSmoother::FixedLagSmoother(fuse_core::Graph::UniquePtr graph,
                                   const ros::NodeHandle& node_handle,
                                   const ros::NodeHandle& private_node_handle)
    : fuse_optimizers::Optimizer(std::move(graph), node_handle,
                                 private_node_handle),
      ignited_(false),
      optimization_request_(false),
      optimization_running_(true),
      started_(false) {
  params_.loadFromROS(private_node_handle);

  // get additional parameter
  bs_parameters::getParam(ros::NodeHandle("~"), "pseudo_marginalization",
                          use_pseudo_marginalization_, false);

  // Test for auto-start
  autostart();

  // Start the optimization thread
  optimization_thread_ = std::thread(&FixedLagSmoother::optimizationLoop, this);

  // Configure a timer to trigger optimizations
  optimize_timer_ =
      node_handle_.createTimer(params_.optimization_period,
                               &FixedLagSmoother::optimizerTimerCallback, this);

  // Advertise a service that resets the optimizer to its initial state
  reset_service_server_ = node_handle_.advertiseService(
      ros::names::resolve(params_.reset_service),
      &FixedLagSmoother::resetServiceCallback, this);

  reset_subscriber_ =
      node_handle_.subscribe(ros::names::resolve(params_.reset_service), 1,
                             &FixedLagSmoother::resetCallback, this);
}

FixedLagSmoother::~FixedLagSmoother() {
  // Wake up any sleeping threads
  optimization_running_ = false;
  optimization_requested_.notify_all();
  // Wait for the threads to shutdown
  if (optimization_thread_.joinable()) { optimization_thread_.join(); }
}

void FixedLagSmoother::autostart() {
  if (std::none_of(sensor_models_.begin(), sensor_models_.end(),
                   [](const auto& element) {
                     return element.second.ignition;
                   })) // NOLINT(whitespace/braces)
  {
    // No ignition sensors were provided. Auto-start.
    started_ = true;
    setStartTime(ros::Time(0, 0));
    ROS_INFO_STREAM("No ignition sensors were specified. Optimization will "
                    "begin immediately.");
  }
}

void FixedLagSmoother::preprocessMarginalization(
    const fuse_core::Transaction& new_transaction) {
  timestamp_tracking_.addNewTransaction(new_transaction);
}

ros::Time FixedLagSmoother::computeLagExpirationTime() const {
  // Find the most recent variable timestamp
  auto start_time = getStartTime();
  auto now = timestamp_tracking_.currentStamp();
  // Then carefully subtract the lag duration. ROS Time objects do not handle
  // negative values.
  return (start_time + params_.lag_duration < now) ? now - params_.lag_duration
                                                   : start_time;
}

std::vector<fuse_core::UUID> FixedLagSmoother::computeVariablesToMarginalize(
    const ros::Time& lag_expiration) {
  auto marginalize_variable_uuids = std::vector<fuse_core::UUID>();
  timestamp_tracking_.query(lag_expiration,
                            std::back_inserter(marginalize_variable_uuids));
  return marginalize_variable_uuids;
}

void FixedLagSmoother::postprocessMarginalization(
    const fuse_core::Transaction& marginal_transaction) {
  timestamp_tracking_.addMarginalTransaction(marginal_transaction);
}

void FixedLagSmoother::optimizationLoop() {
  auto exit_wait_condition = [this]() {
    return this->optimization_request_ || !this->optimization_running_ ||
           !ros::ok();
  };
  // Optimize constraints until told to exit
  while (ros::ok() && optimization_running_) {
    // Wait for the next signal to start the next optimization cycle
    auto optimization_deadline = ros::Time(0, 0);
    {
      std::unique_lock<std::mutex> lock(optimization_requested_mutex_);
      optimization_requested_.wait(lock, exit_wait_condition);
      optimization_request_ = false;
      optimization_deadline = optimization_deadline_;
    }
    // If a shutdown is requested, exit now.
    if (!optimization_running_ || !ros::ok()) { break; }
    // Optimize
    {
      std::lock_guard<std::mutex> lock(optimization_mutex_);
      // Apply motion models
      auto new_transaction = fuse_core::Transaction::make_shared();
      // DANGER: processQueue obtains a lock from the
      // pending_transactions_mutex_
      //         We do this to ensure state of the graph does not change between
      //         unlocking the pending_transactions queue and obtaining the lock
      //         for the graph. But we have now obtained two different locks. If
      //         we are not extremely careful, we could get a deadlock.
      processQueue(*new_transaction, lag_expiration_);
      // Skip this optimization cycle if the transaction is empty because
      // something failed while processing the pending transactions queue.
      if (new_transaction->empty()) { continue; }

      // ! check for invalid constraints
      std::vector<fuse_core::UUID> faulty_constraints;
      for (auto& c : new_transaction->addedConstraints()) {
        for (auto var_uuid : c.variables()) {
          for (auto marginal_uuid : marginal_transaction_.removedVariables()) {
            if (var_uuid == marginal_uuid) {
              faulty_constraints.push_back(c.uuid());
              break;
            }
          }
        }
      }
      if (faulty_constraints.size() > 0) {
        ROS_WARN_STREAM("Removing invalid constraints.");
        for (auto& faulty_constraint : faulty_constraints) {
          new_transaction->removeConstraint(faulty_constraint);
        }
      }

      // apply new transaction
      try {
        graph_->update(*new_transaction);
      } catch (const std::exception& ex) {
        std::string output_path = "/tmp/fixed_lag_smoother_error.log";
        std::ofstream f(output_path);
        f << "Graph:\n";
        graph_->print(f);
        f << "\nTransaction:\n";
        new_transaction->print(f);
        ROS_FATAL_STREAM("Failed to update graph with transaction: "
                         << ex.what()
                         << "\nLeaving optimization loop and requesting "
                            "node shutdown...\n");
        ROS_WARN("outputting graph to: %s", output_path.c_str());
        f.close();
        ros::requestShutdown();
        break;
      }

      // Marginalize variable
      ROS_DEBUG("Marginalizing graph.");
      preprocessMarginalization(*new_transaction);
      lag_expiration_ = computeLagExpirationTime();
      auto vars_to_marginalize = computeVariablesToMarginalize(lag_expiration_);

      if (use_pseudo_marginalization_) {
        marginal_transaction_ = fuse_core::Transaction();
        if (vars_to_marginalize.size() > 1) {
          // remove variables
          for (const auto uuid : vars_to_marginalize) {
            try {
              // remove all connected constraints
              auto constraints = graph_->getConnectedConstraints(uuid);
              for (const auto& c : constraints) {
                marginal_transaction_.removeConstraint(c.uuid());
              }
              marginal_transaction_.removeVariable(uuid);

            } catch (const std::exception& ex) {}
          }

          // add prior on the new start state
          bs_common::ImuState first_window_state = GetWindowStartState();
          fuse_core::Constraint::SharedPtr prior = std::make_shared<
              bs_constraints::AbsoluteImuState3DStampedConstraint>(
              "MARGINALIZATION", first_window_state,
              first_window_state.GetStateVector(),
              Eigen::Matrix<double, 15, 15>::Identity() * 0.00001);
          marginal_transaction_.addConstraint(prior);
        }
      } else {
        marginal_transaction_ = fuse_constraints::marginalizeVariables(
            ros::this_node::getName(), vars_to_marginalize, *graph_);
      }

      graph_->update(marginal_transaction_);
      // Perform any post-marginal cleanup
      postprocessMarginalization(marginal_transaction_);
      ROS_DEBUG("----Done marginalizing fuse graph");

      // Optimize the entire graph
      ROS_DEBUG("Optimizing fuse graph");
      summary_ = graph_->optimize(params_.solver_options);
      ROS_DEBUG("Done optimizing fuse graph");

      // Abort if optimization failed. Not converging is not a failure because
      // the solution found is usable.
      if (!summary_.IsSolutionUsable()) {
        ROS_FATAL_STREAM(
            "Optimization failed after updating the graph with the transaction "
            "with timestamp "
            << new_transaction->stamp()
            << ". Leaving optimization loop and requesting node shutdown...");
        ROS_INFO_STREAM(summary_.FullReport());
        ros::requestShutdown();
        break;
      }

      // Log a warning if the optimization took too long
      auto optimization_complete = ros::Time::now();
      if (optimization_complete > optimization_deadline) {
        ROS_WARN_STREAM_THROTTLE(
            10.0, "Optimization exceeded the configured duration by "
                      << (optimization_complete - optimization_deadline)
                      << "s");
      }

      // Optimization is complete. Notify all the things about the graph
      // changes.
      notify(std::move(new_transaction), graph_->clone());
    }
  }
}

void FixedLagSmoother::optimizerTimerCallback(const ros::TimerEvent& event) {
  // If an "ignition" transaction hasn't been received, then we can't do
  // anything yet.
  if (!started_) { return; }
  // If there is some pending work, trigger the next optimization cycle.
  // If the optimizer has not completed the previous optimization cycle, then it
  // will not be waiting on the condition variable signal, so nothing will
  // happen.
  {
    std::lock_guard<std::mutex> lock(pending_transactions_mutex_);
    optimization_request_ = !pending_transactions_.empty();
  }
  if (optimization_request_) {
    {
      std::lock_guard<std::mutex> lock(optimization_requested_mutex_);
      optimization_deadline_ =
          event.current_expected + params_.optimization_period;
    }
    optimization_requested_.notify_one();
  }
}

void FixedLagSmoother::processQueue(fuse_core::Transaction& transaction,
                                    const ros::Time& lag_expiration) {
  // We need to get the pending transactions from the queue
  std::lock_guard<std::mutex> pending_transactions_lock(
      pending_transactions_mutex_);

  if (pending_transactions_.empty()) { return; }

  // If we just started because an ignition sensor transaction was received, we
  // try to process it individually. This is important because we need to update
  // the graph with the ignition sensor transaction in order to get the motion
  // models notified of the initial state. The motion models will typically
  // maintain a state history in order to create motion model constraints with
  // the optimized variables from the updated graph. If we do not process the
  // ignition sensor transaction individually, the motion model constraints
  // created for the other queued transactions will not be able to use any
  // optimized variables from the graph because it is not been optimized yet,
  // and they will have to use a default zero state instead. This can easily
  // lead to local minima because the variables in the graph are not initialized
  // properly, i.e. they do not take the ignition sensor transaction into
  // account.
  if (ignited_) {
    // The ignition sensor transaction is assumed to be at the end of the queue,
    // because it must be the oldest one. If there is more than one ignition
    // sensor transaction in the queue, it is always the oldest one that started
    // things up.
    ignited_ = false;

    const auto transaction_rbegin = pending_transactions_.rbegin();
    auto& element = *transaction_rbegin;
    if (!sensor_models_.at(element.sensor_name).ignition) {
      // We just started, but the oldest transaction is not from an ignition
      // sensor. We will still process the transaction, but we do not enforce it
      // is processed individually.
      ROS_ERROR_STREAM(
          "The queued transaction with timestamp "
          << element.stamp() << " from sensor " << element.sensor_name
          << " is not an ignition sensor transaction. "
          << "This transaction will not be processed individually.");
    } else {
      if (applyMotionModels(element.sensor_name, *element.transaction)) {
        // Processing was successful. Add the results to the final transaction,
        // delete this one, and return, so the transaction from the ignition
        // sensor is processed individually.
        transaction.merge(*element.transaction, true);
        erase(pending_transactions_, transaction_rbegin);
      } else {
        // The motion model processing failed. When this happens to an ignition
        // sensor transaction there is no point on trying again next time, so we
        // ignore this transaction.
        ROS_ERROR_STREAM(
            "The queued ignition transaction with timestamp "
            << element.stamp() << " from sensor " << element.sensor_name
            << " could not be processed. Ignoring this ignition transaction.");

        // Remove the ignition transaction that just failed and purge all
        // transactions after it. But if we find another ignition transaction,
        // we schedule it to be processed in the next optimization cycle.
        erase(pending_transactions_, transaction_rbegin);

        const auto pending_ignition_transaction_iter = std::find_if(
            pending_transactions_.rbegin(), pending_transactions_.rend(),
            [this](const auto& element) { // NOLINT(whitespace/braces)
              return sensor_models_.at(element.sensor_name).ignition;
            }); // NOLINT(whitespace/braces)
        if (pending_ignition_transaction_iter == pending_transactions_.rend()) {
          // There is no other ignition transaction pending. We simply roll back
          // to not started state and all other pending transactions will be
          // handled later in the transaction callback, as usual.
          started_ = false;
        } else {
          // Erase all transactions before the other ignition transaction
          // pending. This other ignition transaction will be processed in the
          // next optimization cycle.
          pending_transactions_.erase(pending_ignition_transaction_iter.base(),
                                      pending_transactions_.rbegin().base());
          ignited_ = true;
        }
      }

      // There are no more pending transactions to process in this optimization
      // cycle, or they should be processed in the next one.
      return;
    }
  }

  // Use the most recent transaction time as the current time
  const auto current_time = pending_transactions_.front().stamp();

  // Attempt to process each pending transaction
  auto sensor_blacklist = std::vector<std::string>();
  auto transaction_riter = pending_transactions_.rbegin();
  while (transaction_riter != pending_transactions_.rend()) {
    auto& element = *transaction_riter;
    const auto& min_stamp = element.minStamp();
    if (min_stamp < lag_expiration) {
      ROS_DEBUG_STREAM(
          "The current lag expiration time is "
          << lag_expiration
          << ". The queued transaction with "
             "timestamp "
          << element.stamp() << " from sensor " << element.sensor_name
          << " has a minimum "
             "involved timestamp of "
          << min_stamp << ", which is " << (lag_expiration - min_stamp)
          << " seconds too old. Ignoring this transaction.");
      transaction_riter = erase(pending_transactions_, transaction_riter);
    } else if (std::find(sensor_blacklist.begin(), sensor_blacklist.end(),
                         element.sensor_name) != sensor_blacklist.end()) {
      // We should not process transactions from this sensor
      ++transaction_riter;
    } else if (applyMotionModels(element.sensor_name, *element.transaction)) {
      // Processing was successful. Add the results to the final transaction,
      // delete this one, and move to the next.
      transaction.merge(*element.transaction, true);
      transaction_riter = erase(pending_transactions_, transaction_riter);
    } else {
      // The motion model processing failed.
      // Check the transaction timeout to determine if it should be removed or
      // skipped.
      const auto& max_stamp = element.maxStamp();
      if (max_stamp + params_.transaction_timeout < current_time) {
        // Warn that this transaction has expired, then skip it.
        ROS_ERROR_STREAM(
            "The queued transaction with timestamp "
            << element.stamp()
            << " and maximum "
               "involved stamp of "
            << max_stamp << " from sensor " << element.sensor_name
            << " could not be processed after " << (current_time - max_stamp)
            << " seconds, "
               "which is greater than the 'transaction_timeout' value of "
            << params_.transaction_timeout << ". Ignoring this transaction.");
        transaction_riter = erase(pending_transactions_, transaction_riter);
      } else {
        // The motion model failed. Stop further processing of this sensor and
        // try again next time.
        sensor_blacklist.push_back(element.sensor_name);
        ++transaction_riter;
      }
    }
  }
}

bool FixedLagSmoother::resetServiceCallback(std_srvs::Empty::Request&,
                                            std_srvs::Empty::Response&) {
  ROS_ERROR_STREAM("Reset service received! Resetting system...");
  // Tell all the plugins to stop
  stopPlugins();
  // Reset the optimizer state
  optimization_request_ = false;
  started_ = false;
  ignited_ = false;
  setStartTime(ros::Time(0, 0));
  // DANGER: The optimizationLoop() function obtains the lock
  // optimization_mutex_ lock and the
  //         pending_transactions_mutex_ lock at the same time. We perform a
  //         parallel locking scheme here to prevent the possibility of
  //         deadlocks.
  {
    std::lock_guard<std::mutex> lock(optimization_mutex_);
    // Clear all pending transactions
    {
      std::lock_guard<std::mutex> lock(pending_transactions_mutex_);
      pending_transactions_.clear();
    }
    // Clear the graph and marginal tracking states
    graph_->clear();
    marginal_transaction_ = fuse_core::Transaction();
    timestamp_tracking_.clear();
    lag_expiration_ = ros::Time(0, 0);
  }
  // Tell all the plugins to start
  startPlugins();
  // Test for auto-start
  autostart();

  return true;
}

void FixedLagSmoother::resetCallback(const std_msgs::Empty::ConstPtr&) {
  ROS_ERROR_STREAM("Reset callback received! Resetting system...");
  // Tell all the plugins to stop
  stopPlugins();
  // Reset the optimizer state
  optimization_request_ = false;
  started_ = false;
  ignited_ = false;
  setStartTime(ros::Time(0, 0));
  // DANGER: The optimizationLoop() function obtains the lock
  // optimization_mutex_ lock and the
  //         pending_transactions_mutex_ lock at the same time. We perform a
  //         parallel locking scheme here to prevent the possibility of
  //         deadlocks.
  {
    std::lock_guard<std::mutex> lock(optimization_mutex_);
    // Clear all pending transactions
    {
      std::lock_guard<std::mutex> lock(pending_transactions_mutex_);
      pending_transactions_.clear();
    }
    // Clear the graph and marginal tracking states
    graph_->clear();
    marginal_transaction_ = fuse_core::Transaction();
    timestamp_tracking_.clear();
    lag_expiration_ = ros::Time(0, 0);
  }
  // Tell all the plugins to start
  startPlugins();
  // Test for auto-start
  autostart();
}

void FixedLagSmoother::transactionCallback(
    const std::string& sensor_name,
    fuse_core::Transaction::SharedPtr transaction) {
  // If this transaction occurs before the start time, just ignore it
  auto start_time = getStartTime();
  const auto max_time = transaction->maxStamp();
  if (started_ && max_time < start_time) {
    ROS_DEBUG_STREAM(
        "Received a transaction before the start time from sensor '"
        << sensor_name << "'.\n"
        << "  start_time: " << start_time << ", maximum involved stamp: "
        << max_time << ", difference: " << (start_time - max_time) << "s");
    return;
  }
  {
    // We need to add the new transaction to the pending_transactions_ queue
    std::lock_guard<std::mutex> pending_transactions_lock(
        pending_transactions_mutex_);

    // Add the new transaction to the pending set
    // The pending set is arranged "smallest stamp last" to making popping off
    // the back more efficient
    auto comparator = [](const ros::Time& value,
                         const TransactionQueueElement& element) {
      return value >= element.stamp();
    };
    auto position = std::upper_bound(pending_transactions_.begin(),
                                     pending_transactions_.end(),
                                     transaction->stamp(), comparator);
    position = pending_transactions_.insert(
        position, {sensor_name, std::move(transaction)}); // NOLINT

    // If we haven't "started" yet..
    if (!started_) {
      // ...check if we should
      if (sensor_models_.at(sensor_name).ignition) {
        started_ = true;
        ignited_ = true;
        start_time = position->minStamp();
        setStartTime(start_time);

        // And purge out old transactions
        //  - Either before or exactly at the start time
        //  - Or with a minimum time before the minimum time of this ignition
        //  sensor transaction
        //
        // TODO(efernandez) Do '&min_time = std::as_const(start_ime)' when C++17
        // is supported and we can use
        //                  std::as_const:
        //                  https://en.cppreference.com/w/cpp/utility/as_const
        pending_transactions_.erase(
            std::remove_if(
                pending_transactions_.begin(), pending_transactions_.end(),
                [&sensor_name, max_time,
                 &min_time = start_time](
                    const auto& transaction) { // NOLINT(whitespace/braces)
                  return transaction.sensor_name != sensor_name &&
                         (transaction.minStamp() < min_time ||
                          transaction.maxStamp() <= max_time);
                }), // NOLINT(whitespace/braces)
            pending_transactions_.end());
      } else {
        // And purge out old transactions to limit the pending size while
        // waiting for an ignition sensor
        auto purge_time = ros::Time(0, 0);
        auto last_pending_time = pending_transactions_.front().stamp();
        if (ros::Time(0, 0) + params_.transaction_timeout <
            last_pending_time) // ros::Time doesn't allow negatives
        {
          purge_time = last_pending_time - params_.transaction_timeout;
        }

        while (!pending_transactions_.empty() &&
               pending_transactions_.back().maxStamp() < purge_time) {
          pending_transactions_.pop_back();
        }
      }
    }
  }
}

/**
 * @brief Make a diagnostic_msgs::DiagnosticStatus message filling in the level
 * and message
 *
 * @param[in] level   The diagnostic status level
 * @param[in] message The diagnostic status message
 */
diagnostic_msgs::DiagnosticStatus
    makeDiagnosticStatus(const int8_t level, const std::string& message) {
  diagnostic_msgs::DiagnosticStatus status;

  status.level = level;
  status.message = message;

  return status;
}

/**
 * @brief Helper function to generate the diagnostic status for each
 * optimization termination type
 *
 * The termination type -> diagnostic status mapping is as follows:
 *
 * - CONVERGENCE, USER_SUCCESS -> OK
 * - NO_CONVERGENCE            -> WARN
 * - FAILURE, USER_FAILURE     -> ERROR (default)
 *
 * @param[in] termination_type The optimization termination type
 * @return The diagnostic status with the level and message corresponding to the
 * optimization termination type
 */
diagnostic_msgs::DiagnosticStatus terminationTypeToDiagnosticStatus(
    const ceres::TerminationType termination_type) {
  switch (termination_type) {
    case ceres::TerminationType::CONVERGENCE:
    case ceres::TerminationType::USER_SUCCESS:
      return makeDiagnosticStatus(diagnostic_msgs::DiagnosticStatus::OK,
                                  "Optimization converged");
    case ceres::TerminationType::NO_CONVERGENCE:
      return makeDiagnosticStatus(diagnostic_msgs::DiagnosticStatus::WARN,
                                  "Optimization didn't converge");
    default:
      return makeDiagnosticStatus(diagnostic_msgs::DiagnosticStatus::ERROR,
                                  "Optimization failed");
  }
}

void FixedLagSmoother::setDiagnostics(
    diagnostic_updater::DiagnosticStatusWrapper& status) {
  Optimizer::setDiagnostics(status);

  // Load std::atomic<bool> flag that indicates whether the optimizer has
  // started or not
  const bool started = started_;

  status.add("Started", started);
  {
    std::lock_guard<std::mutex> lock(pending_transactions_mutex_);
    status.add("Pending Transactions", pending_transactions_.size());
  }

  if (started) {
    // Add some optimization summary report fields to the diagnostics status if
    // the optimizer has started
    auto summary = decltype(summary_)();
    {
      const std::unique_lock<std::mutex> lock(optimization_mutex_,
                                              std::try_to_lock);
      if (lock) {
        summary = summary_;
      } else {
        status.summary(diagnostic_msgs::DiagnosticStatus::OK,
                       "Optimization running");
      }
    }

    if (summary.total_time_in_seconds >=
        0.0) // This is -1 for the default-constructed summary object
    {
      status.add("Optimization Termination Type",
                 ceres::TerminationTypeToString(summary.termination_type));
      status.add("Optimization Total Time [s]", summary.total_time_in_seconds);
      status.add("Optimization Iterations", summary.iterations.size());
      status.add("Initial Cost", summary.initial_cost);
      status.add("Final Cost", summary.final_cost);

      status.mergeSummary(
          terminationTypeToDiagnosticStatus(summary.termination_type));
    }

    // Add time since the last optimization request time. This is useful to
    // detect if no transactions are received for too long
    auto optimization_deadline = decltype(optimization_deadline_)();
    {
      const std::unique_lock<std::mutex> lock(optimization_requested_mutex_,
                                              std::try_to_lock);
      if (lock) { optimization_deadline = optimization_deadline_; }
    }

    if (!optimization_deadline
             .isZero()) // This is zero for the default-constructed
                        // optimization_deadline object
    {
      const auto optimization_request_time =
          optimization_deadline - params_.optimization_period;
      const auto time_since_last_optimization_request =
          ros::Time::now() - optimization_request_time;
      status.add("Time Since Last Optimization Request [s]",
                 time_since_last_optimization_request.toSec());
    }
  }
}

bs_common::ImuState FixedLagSmoother::GetWindowStartState() {
  // retrieve the timestamp of the first state in the window
  ros::Time first_window_time(ros::TIME_MAX);
  for (const auto& v : graph_->getVariables()) {
    if (v.type() != "fuse_variables::Position3DStamped") continue;
    auto position = fuse_variables::Position3DStamped::make_shared();
    *position = dynamic_cast<const fuse_variables::Position3DStamped&>(v);
    if (position->stamp() > lag_expiration_ &&
        position->stamp() < first_window_time) {
      first_window_time = position->stamp();
    }
  }
  bs_common::ImuState first_window_state(first_window_time);

  try {
    // get position
    auto position = fuse_variables::Position3DStamped::make_shared();
    const auto pos_uuid = fuse_core::uuid::generate(
        position->type(), first_window_time, fuse_core::uuid::NIL);
    *position = dynamic_cast<const fuse_variables::Position3DStamped&>(
        graph_->getVariable(pos_uuid));
    // get orientation
    auto orientation = fuse_variables::Orientation3DStamped::make_shared();
    const auto or_uuid = fuse_core::uuid::generate(
        orientation->type(), first_window_time, fuse_core::uuid::NIL);
    *orientation = dynamic_cast<const fuse_variables::Orientation3DStamped&>(
        graph_->getVariable(or_uuid));
    // get velocity
    auto velocity = fuse_variables::VelocityLinear3DStamped::make_shared();
    const auto vel_uuid = fuse_core::uuid::generate(
        velocity->type(), first_window_time, fuse_core::uuid::NIL);
    *velocity = dynamic_cast<const fuse_variables::VelocityLinear3DStamped&>(
        graph_->getVariable(vel_uuid));
    // get accel bias
    auto accel_bias = bs_variables::AccelerationBias3DStamped::make_shared();
    const auto ba_uuid = fuse_core::uuid::generate(
        accel_bias->type(), first_window_time, fuse_core::uuid::NIL);
    *accel_bias = dynamic_cast<const bs_variables::AccelerationBias3DStamped&>(
        graph_->getVariable(ba_uuid));
    // get gyro bias
    auto gyro_bias = bs_variables::GyroscopeBias3DStamped::make_shared();
    const auto bg_uuid = fuse_core::uuid::generate(
        gyro_bias->type(), first_window_time, fuse_core::uuid::NIL);
    *gyro_bias = dynamic_cast<const bs_variables::GyroscopeBias3DStamped&>(
        graph_->getVariable(bg_uuid));
    first_window_state.SetPosition(*position);
    first_window_state.SetOrientation(*orientation);
    first_window_state.SetVelocity(*velocity);
    first_window_state.SetAccelBias(*accel_bias);
    first_window_state.SetGyroBias(*gyro_bias);
    return first_window_state;
  } catch (const std::out_of_range& oor) {
    throw std::out_of_range(
        "Invalid start state of new graph, not all imu state variables exist.");
  }
}

} // namespace bs_optimizers
