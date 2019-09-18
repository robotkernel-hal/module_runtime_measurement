//! robotkernel module runtime_measurement
/*!
 * author: Robert Burger <robert.burger@dlr.de>
 */

/*
 * This file is part of robotkernel.
 *
 * robotkernel is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * robotkernel is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with robotkernel.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "runtime_measurement.h"
#include "robotkernel/helpers.h"
#include "config.h"

#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>
#include <iostream>
#include <semaphore.h>
#include <signal.h>
#include <math.h>

#include "yaml-cpp/yaml.h"
#include <string_util/string_util.h>

MODULE_DEF(module_runtime_measurement, module_runtime_measurement::runtime_measurement)

using namespace std;
using namespace std::placeholders;
using namespace robotkernel;
using namespace module_runtime_measurement;
using namespace string_util;
        
runtime_measurement::msr_path::msr_path(runtime_measurement& parent, const YAML::Node& node) :
    parent(parent)
{
    dev_name              = get_as<std::string>(node, "trigger_dev_name");
    msr_path_name         = get_as<std::string>(node, "name");
    buffer_size           = get_as<unsigned int>(node, "buffer_size", 1000);
    buffer_pos            = 0;
    buffer_act            = 0;

    // resize buffers
    log_dur[0].resize(buffer_size);
    log_dur[1].resize(buffer_size);
}
                
runtime_measurement::msr_path::~msr_path() {};

void runtime_measurement::msr_path::start() {
    robotkernel::kernel& k = *robotkernel::kernel::get_instance();

    // get/create triggers
    input_t_dev = k.get_trigger(dev_name);
    slave_t_dev = make_shared<runtime_measurement::slave_trigger>(input_t_dev, parent.name, 
            format_string("%s", msr_path_name.c_str()));
    k.add_device(slave_t_dev);
    
    input_t_dev->add_trigger(shared_from_this());

    runnable::start();
}

void runtime_measurement::msr_path::stop() {
    robotkernel::kernel& k = *robotkernel::kernel::get_instance();

    runnable::stop();

    input_t_dev->remove_trigger(shared_from_this());

    k.remove_device(slave_t_dev);
    slave_t_dev = nullptr;
    input_t_dev = nullptr;
}

void runtime_measurement::msr_path::tick() {
    // get actual duration
    auto begin = std::chrono::high_resolution_clock::now();
    slave_t_dev->trigger_modules();
    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<uint64_t, std::nano> duration = end - begin;
    uint64_t ns_duration = duration.count();
    
    log_dur[buffer_act][buffer_pos++] = ns_duration;
    if (buffer_pos >= buffer_size) {
        buffer_pos = 0;
        buffer_act = (buffer_act + 1) % 2; 

        sync_cond.notify_one();
    }
}

//! handler function called if thread is running
void runtime_measurement::msr_path::run() {
    std::unique_lock<std::mutex> lock(sync_mtx);

    while (running()) {
        if (sync_cond.wait_for(lock, std::chrono::seconds(1))
                == std::cv_status::timeout)
            continue;

        // print statistic
        log_dur_vec_t& act_buf = log_dur[(buffer_act + 1) % 2];

        uint64_t dev;
        uint64_t avg_dur = 0, avgjit = 0, maxjit = 0;

        // calculate differences and sum of differences
        for (unsigned i = 0; i < buffer_size; ++i) {
            avg_dur += act_buf[i];
            maxjit  = max(act_buf[i], maxjit);
        }
        avg_dur /= buffer_size;

        // calculating maximum deviation
        for (unsigned i = 0; i < buffer_size; i++) {
            dev     = abs((int64_t)act_buf[i] - (int64_t)avg_dur); 

            avgjit += (dev * dev);
        }

        avgjit = sqrt(avgjit/(buffer_size - 1));
        
        parent.log(info, "%s: mean duration: %4.0lfus, jitter mean:"
                " %4.0lfus, max %4.0lfus\n", msr_path_name.c_str(),
                (double)avg_dur / 1E3, (double)avgjit / 1E3, (double)maxjit / 1E3);
    }
}

//! yaml config construction
/*!
 * \param name name of jm
 * \param node yaml node
 */
runtime_measurement::runtime_measurement(const char* name, const YAML::Node& node) :
    module_base("module_runtime_measurement", name, node) 
{
    for (const auto& tr_node : node["triggers"]) {
        auto tmp_path = make_shared<msr_path>(*this, tr_node);
        msr_path_map[tmp_path->msr_path_name] = tmp_path;
    }
}

//! default destruction
runtime_measurement::~runtime_measurement() {
    // destroy trigger device
    msr_path_map.clear();
}

int runtime_measurement::set_state(module_state_t state) {
    // get transition
    uint32_t transition = GEN_STATE(this->state, state);
    
    log(info, "state %s requested\n", state_to_string(state));

    switch (transition) {
        case op_2_safeop:
        case op_2_preop:
        case op_2_init:
        case op_2_boot:
            // ====> stop sending commands

            if (state == module_state_safeop)
                break;
        case safeop_2_preop:
        case safeop_2_init:
        case safeop_2_boot:
            // ====> stop receiving measurements
            for (auto& kv : msr_path_map)
                kv.second->stop();

            if (state == module_state_preop)
                break;
        case preop_2_init:
        case preop_2_boot:
            // ====> deinit devices
        case init_2_init:
            // ====> re-/open device
            if (state == module_state_init)
                break;
        case init_2_boot:
            break;
        case boot_2_init:
        case boot_2_preop:
        case boot_2_safeop:
        case boot_2_op:
            // ====> re-/open device
            if (state == module_state_init)
                break;
        case init_2_op:
        case init_2_safeop:
        case init_2_preop:
            // ====> initial devices            
            if (state == module_state_preop)
                break;
        case preop_2_op:
        case preop_2_safeop:
            // ====> start receiving measurements
            for (auto& kv : msr_path_map)
                kv.second->start();

            if (state == module_state_safeop)
                break;
        case safeop_2_op:
            // ====> start sending commands
            break;
        case op_2_op:
        case safeop_2_safeop:
        case preop_2_preop:
            // ====> do nothing
            break;

        default:
            break;
    }

    return (this->state = state);
}
        
