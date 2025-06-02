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

#ifndef __MODULE_RUNTIME_MEASSUREMENT_H__
#define __MODULE_RUNTIME_MEASSUREMENT_H__

#include "robotkernel/runnable.h"
#include "robotkernel/module_base.h"

#include "service_provider/process_data_inspection/base.h"

#include "yaml-cpp/yaml.h"

#include <mutex>
#include <chrono>
#include <condition_variable>

#include "config.h"

namespace module_runtime_measurement {
#ifdef EMACS
}
#endif

class runtime_measurement :
    public std::enable_shared_from_this<runtime_measurement>,
    public robotkernel::module_base
{
    
    public:
        class slave_trigger : public robotkernel::trigger {
            public:
                robotkernel::sp_trigger_t master_trigger;

                slave_trigger(robotkernel::sp_trigger_t master_trigger, 
                        const std::string& owner, const std::string& name) :
                    robotkernel::trigger(owner, name, master_trigger->get_rate()),
                    master_trigger(master_trigger) {}

                //! set rate of trigger 
                /*!
                 * set the rate of the current trigger
                 * overload in derived trigger class
                 *
                 * \param new_rate new trigger rate to set
                 */
                void set_rate(double new_rate) {
                    master_trigger->set_rate(new_rate);
                }
        };

        class msr_path : 
            public robotkernel::trigger_base,
            public robotkernel::runnable,
            public std::enable_shared_from_this<msr_path>
        {
            public:
                runtime_measurement& parent;

                std::string dev_name;
                std::string msr_path_name;
                robotkernel::sp_trigger_t input_t_dev, slave_t_dev;

                msr_path(runtime_measurement& parent, const YAML::Node& node);
                ~msr_path();

                void start();
                void stop();

                void tick();
        
                //! handler function called if thread is running
                void run();

                // input process data
                struct runtime_pdin {
                    uint64_t last_dur;
                };

                // named process data
                robotkernel::sp_process_data_t runtime_pdin;
                robotkernel::sp_pd_provider_t runtime_prov;
                service_provider::process_data_inspection::sp_pd_inspection_t runtime_pdin_inspect;

                //! print thread sync
                std::mutex              sync_mtx;
                std::condition_variable sync_cond;

        
                size_t buffer_size;       //! size of runtime measurement buffer
                //! position in buffer
                unsigned int buffer_act;
                unsigned int buffer_pos;

                typedef std::vector<uint64_t> log_dur_vec_t;
                log_dur_vec_t log_dur[2];
        };
            
    private: 
        std::map<std::string, std::shared_ptr<msr_path> > msr_path_map;

    public:
        //! yaml config construction
        /*!
         * \param name of jm 
         * \param node yaml node
         */
        runtime_measurement(const char* name, const YAML::Node& node);

        //! default destruction
        ~runtime_measurement();

        //! additional init function
        void init() {}

        //! set module state
        /*
         * \param state module state to set
         * \return 0 on success
         */
        int set_state(module_state_t state);

        //! module trigger callback
        /*! does one measurement
         *
         * if log buffer is full, output thread is triggered
         */
        void tick() {}
};

#ifdef EMACS
{
#endif
};

#endif // __MODULE_RUNTIME_MEASSUREMENT_H__

