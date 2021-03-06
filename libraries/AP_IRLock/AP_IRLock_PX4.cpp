/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * AP_IRLock_PX4.cpp
 *
 *  Created on: Nov 16, 2014
 *      Author: MLandes
 */

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include "AP_IRLock_PX4.h"

#include <fcntl.h>
#include <unistd.h>

#include "AP_HAL.h"
#include "drivers/drv_irlock.h"

extern const AP_HAL::HAL& hal;

AP_IRLock_PX4::AP_IRLock_PX4() :
		_fd(0),
		_last_timestamp(0)
{}

void AP_IRLock_PX4::init()
{
	_fd = open(IRLOCK0_DEVICE_PATH, O_RDONLY);
	if (_fd < 0) {
		hal.console->printf("Unable to open " IRLOCK0_DEVICE_PATH "\n");
		return;
	}

	_flags.healthy = true;
}

// retrieve latest sensor data - returns true if new data is available
bool AP_IRLock_PX4::update()
{
    // return immediately if not healthy
	if (!_flags.healthy) {
		return false;
	}

	// read position of all objects
	struct irlock_s report;
	uint16_t count = 0;
	while(::read(_fd, &report, sizeof(struct irlock_s)) == sizeof(struct irlock_s) && report.timestamp >_last_timestamp) {
//TMS	    _current_frame[count].center_x = report.center_x;
//TMS		_current_frame[count].center_y = report.center_y;
        _current_frame[count].angle_x = report.angle_x; //TMSnote this is an angle
        _current_frame[count].angle_y = report.angle_y; //TMSnote this is an angle
        //hal.console->printf("report_angle_x: %f\n", report.angle_x);
        //hal.console->printf("_current_frame_angle_x: %f\n\n", _current_frame[count].angle_x);
//TMS		_current_frame[count].height = report.height;
//TMS        _current_frame[count].width = report.width;
        _current_frame[count].size_x = report.size_x;
        _current_frame[count].size_y = report.size_y;

		count++;
		_last_timestamp = report.timestamp;
		_last_update = hal.scheduler->millis();
	}

	// update num_blocks and implement timeout
	if (count > 0) {
	    //ghm1
	    //hal.console->printf("count: %u\n", count);
	    _num_blocks = count;
	} else if ((hal.scheduler->millis() - _last_update) > IRLOCK_TIMEOUT_MS) {
	    _num_blocks = 0;
	}

	// return true if new data found
	return (_num_blocks > 0);
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_PX4
