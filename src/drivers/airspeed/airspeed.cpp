/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ets_airspeed.cpp
 * @author Simon Wilks
 *
 * Driver for the Eagle Tree Airspeed V3 connected via I2C.
 */

#include <nuttx/config.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>

#include <systemlib/airspeed.h>
#include <systemlib/err.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>

#include <drivers/drv_airspeed.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/differential_pressure_raw_data.h>
#include <uORB/topics/subsystem_info.h>

#include <drivers/airspeed/airspeed.h>

Airspeed::Airspeed(int bus, int address, unsigned conversion_interval, const char* path) :
	I2C("Airspeed", path, bus, address, 100000),
    _reports_processed(nullptr),
	_reports_raw(nullptr),
	_buffer_overflows(perf_alloc(PC_COUNT, "airspeed_buffer_overflows")),
	_max_differential_pressure_pa(0),
	_sensor_ok(false),
	_measure_ticks(0),
	_collect_phase(false),
	_diff_pres_offset(0.0f),
    _diff_pres_scale(0.0f),
    _process_data(true),
    _airspeed_processed_pub(-1),
    _airspeed_raw_pub(-1),
	_class_instance(-1),
	_conversion_interval(conversion_interval),
	_sample_perf(perf_alloc(PC_ELAPSED, "airspeed_read")),
	_comms_errors(perf_alloc(PC_COUNT, "airspeed_comms_errors"))
{
	// enable debug() calls
	_debug_enabled = false;

	// work_cancel in the dtor will explode if we don't do this...
	memset(&_work, 0, sizeof(_work));
}

Airspeed::~Airspeed()
{
	/* make sure we are truly inactive */
	stop();

	if (_class_instance != -1) {
		unregister_class_devname(AIRSPEED_DEVICE_PATH, _class_instance);
    }

	/* free any existing reports */
	if (_reports_raw != nullptr) {
		delete _reports_raw;
    }
    if (_reports_processed != nullptr) {
        delete _reports_processed;
    }

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_buffer_overflows);
}

int Airspeed::init() {
	
    /* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		return ERROR;
    }
    
    _diff_pres_scale = get_default_scale();
    int ret = OK;
    if (_process_data) {
    printf("Init running\n");
        ret = init_processed();
    printf("Init complete\n");
    } else {
        ret = init_raw();
    }
	/* sensor is ok, but we don't really know if it is within range */
	_sensor_ok = true;

	return ret;
}

int Airspeed::init_processed() {
    int ret = ERROR;
    _reports_processed = new RingBuffer(2, sizeof(differential_pressure_s));
    if (_reports_processed == nullptr) {
       goto out;
    }
    _class_instance = register_class_devname(AIRSPEED_DEVICE_PATH);
    // Never getting to here. register_class_devname not working.
    
    /* publication init */
    if (_class_instance == CLASS_DEVICE_PRIMARY) {
        /* advertise sensor topic, measure manually to initialize valid report */
        struct differential_pressure_s arp;
        measure();
        _reports_processed->get(&arp);
        /* measurement will have generated a report, publish */
        _airspeed_processed_pub = orb_advertise(ORB_ID(differential_pressure), &arp);

        if (_airspeed_processed_pub < 0) {
            warnx("failed to create airspeed sensor object. uORB started?");
        }
    }
    printf("Done\n");
    ret = OK;
out:
    return ret;
}

int Airspeed::deinit_processed() {
    if (_reports_processed != nullptr) {
        delete _reports_processed;
    }
    if (_class_instance != -1) {
		unregister_class_devname(AIRSPEED_DEVICE_PATH, _class_instance);
    }
    close((file*) _airspeed_processed_pub);
    return OK;
}

int Airspeed::deinit_raw() {
    if (_reports_raw != nullptr) {
        delete _reports_raw;
    }
    if (_class_instance != -1) {
		unregister_class_devname(AIRSPEED_DEVICE_PATH, _class_instance);
    }
    close((file*) _airspeed_raw_pub);
    return OK;
}

int Airspeed::init_raw() {
    int ret = ERROR;
    _reports_raw = new RingBuffer(2, sizeof(differential_pressure_raw_data_s));
    if (_reports_raw == nullptr) {
       goto out;
    }
    _class_instance = register_class_devname(AIRSPEED_DEVICE_PATH);
    /* publication init */
    if (_class_instance == CLASS_DEVICE_PRIMARY) {

        /* advertise sensor topic, measure manually to initialize valid report */
        struct differential_pressure_raw_data_s arp;
        measure();
        _reports_processed->get(&arp);

        /* measurement will have generated a report, publish */
        _airspeed_raw_pub = orb_advertise(ORB_ID(differential_pressure_raw_data), &arp);

        if (_airspeed_raw_pub < 0) {
            warnx("failed to create airspeed raw data sensor object. uORB started?");
        }
    }
    ret = OK;
out:
    return ret;
}


int
Airspeed::probe()
{
	/* on initial power up the device needs more than one retry
	   for detection. Once it is running then retries aren't
	   needed 
	*/
	_retries = 4;
	int ret = measure();
	_retries = 2;
	return ret;
}

int
Airspeed::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

				/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

				/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

				/* zero would be bad */
			case 0:
				return -EINVAL;

				/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(_conversion_interval);

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}

				/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(_conversion_interval))
						return -EINVAL;

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start)
						start();

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
        }

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100))
				return -EINVAL;

			irqstate_t flags = irqsave();
            if (_process_data) {
                if (!_reports_processed->resize(arg)) {
                    irqrestore(flags);
                    return -ENOMEM;
                }
            } else {
                if (!_reports_raw->resize(arg)) {
                    irqrestore(flags);
                    return -ENOMEM;
                }
            }
			irqrestore(flags);

			return OK;
		}

	case SENSORIOCGQUEUEDEPTH:
        if (_process_data) {
            return _reports_processed->size();
        } else {
            return _reports_raw->size();
        }

	case SENSORIOCRESET:
		/* XXX implement this */
		return -EINVAL;

	case AIRSPEEDIOCSSCALE: {
		struct airspeed_scale *s = (struct airspeed_scale*)arg;
		_diff_pres_offset = s->offset_pa;
        if (s->scale != 0) {
            _diff_pres_scale  = s->scale;
        } else {
            _diff_pres_scale = get_default_scale();
        }
        // We don't flush the raw reports because they're still valid.
        if (_reports_processed != nullptr) {
            _reports_processed->flush();
        }
		return OK;
		}

	case AIRSPEEDIOCGSCALE: {
		struct airspeed_scale *s = (struct airspeed_scale*)arg;
		s->offset_pa = _diff_pres_offset;
		s->scale = _diff_pres_scale;
		return OK;
		}

    case AIRSPEEDIOCSPROCDATA: {
        int retval = OK;
        if (!_process_data && (arg & AIRSPEED_PROCESS_DATA)) {
            if (init_processed() == ERROR) {
                retval = ERROR;
            } else {
                // Initialisation complete.
                _process_data = true;
                deinit_raw(); 
            }
        } else if (_process_data && (!(arg & AIRSPEED_PROCESS_DATA))) {
            if (init_raw() == ERROR) {
                retval = ERROR;
            } else {
                _process_data = false;
                deinit_processed();
            }
        }
        return retval;
        }
        
	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}
ssize_t
Airspeed::read(struct file *filp, char *buffer, size_t buflen)
{
    if (_process_data) {
        return read_processed(filp,buffer,buflen);
    } else {
        return read_raw(filp,buffer,buflen);
    }
}

ssize_t Airspeed::read_processed(struct file * filp, char * buffer, size_t buflen) {
    unsigned int count = buflen / sizeof(differential_pressure_s);
    differential_pressure_s *abuf = reinterpret_cast<differential_pressure_s *>(buffer);
    int ret = 0;

    /* buffer must be large enough */
    if (count < 1) {
        return -ENOSPC;
    }

    /* if automatic measurement is enabled */
    if (_measure_ticks > 0) {

        /*
         * While there is space in the caller's buffer, and reports, copy them.
         * Note that we may be pre-empted by the workq thread while we are doing this;
         * we are careful to avoid racing with them.
         */
        while (count--) {
            if (_reports_processed->get(abuf)) {
                ret += sizeof(*abuf);
                abuf++;
            }
        }

        /* if there was no data, warn the caller */
        return ret ? ret : -EAGAIN;
    }

    /* manual measurement - run one conversion */
    do {
        _reports_processed->flush();

        /* trigger a measurement */
        if (OK != measure()) {
            ret = -EIO;
            break;
        }

        /* wait for it to complete */
        usleep(_conversion_interval);

        /* run the collection phase */
        if (OK != collect()) {
            ret = -EIO;
            break;
        }

        /* state machine will have generated a report, copy it out */
        if (_reports_processed->get(abuf)) {
            ret = sizeof(*abuf);
        }

    } while (0);
    return ret;
}

ssize_t Airspeed::read_raw(struct file * filp, char * buffer, size_t buflen) {
    unsigned int count = buflen / sizeof(differential_pressure_raw_data_s);
    differential_pressure_raw_data_s *abuf = reinterpret_cast<differential_pressure_raw_data_s *>(buffer);
    int ret = 0;

    /* buffer must be large enough */
    if (count < 1) {
        return -ENOSPC;
    }

    /* if automatic measurement is enabled */
    if (_measure_ticks > 0) {

        /*
         * While there is space in the caller's buffer, and reports, copy them.
         * Note that we may be pre-empted by the workq thread while we are doing this;
         * we are careful to avoid racing with them.
         */
        while (count--) {
            if (_reports_raw->get(abuf)) {
                ret += sizeof(*abuf);
                abuf++;
            }
        }

        /* if there was no data, warn the caller */
        return ret ? ret : -EAGAIN;
    }

    /* manual measurement - run one conversion */
    do {
        _reports_raw->flush();

        /* trigger a measurement */
        if (OK != measure()) {
            ret = -EIO;
            break;
        }

        /* wait for it to complete */
        usleep(_conversion_interval);

        /* run the collection phase */
        if (OK != collect()) {
            ret = -EIO;
            break;
        }

        /* state machine will have generated a report, copy it out */
        if (_reports_raw->get(abuf)) {
            ret = sizeof(*abuf);
        }

    } while (0);
    return ret;
}


void
Airspeed::start()
{
	/* reset the report ring and state machine */
	_collect_phase = false;
    if (_process_data) {
        _reports_processed->flush();
    } else {
        _reports_raw->flush();
    }

	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&Airspeed::cycle_trampoline, this, 1);

	/* notify about state change */
	struct subsystem_info_s info = {
		true,
		true,
		true,
		SUBSYSTEM_TYPE_DIFFPRESSURE
	};
	static orb_advert_t pub = -1;

	if (pub > 0) {
		orb_publish(ORB_ID(subsystem_info), pub, &info);
	} else {
		pub = orb_advertise(ORB_ID(subsystem_info), &info);
	}
}

void
Airspeed::stop()
{
	work_cancel(HPWORK, &_work);
}

void
Airspeed::cycle_trampoline(void *arg)
{
	Airspeed *dev = (Airspeed *)arg;

	dev->cycle();
}

void
Airspeed::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_buffer_overflows);
	warnx("poll interval:  %u ticks", _measure_ticks);
    if (_process_data) {
        _reports_processed->print_info("report queue");
    } else {
        _reports_raw->print_info("report queue");
    }
}

void
Airspeed::new_report_processed(const differential_pressure_s &report)
{
	if (!_reports_processed->force(&report))
		perf_count(_buffer_overflows);
}

void
Airspeed::new_report_raw(const differential_pressure_raw_data_s &report)
{
	if (!_reports_raw->force(&report))
		perf_count(_buffer_overflows);
}
