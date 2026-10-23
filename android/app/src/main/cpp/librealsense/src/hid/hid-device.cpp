// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#include <queue>
#include <algorithm>
#include "hid-device.h"

const int USB_REQUEST_COUNT = 4;

namespace librealsense
{
    namespace platform
    {
        std::vector<hid_device_info> query_hid_devices_info()
        {
            std::vector<std::string> hid_sensors = { gyro, accel, custom };

            std::vector<hid_device_info> rv;
            auto usb_devices = platform::usb_enumerator::query_devices_info();
            for (auto&& info : usb_devices) {
                if(info.cls != RS2_USB_CLASS_HID)
                    continue;
                platform::hid_device_info device_info;
                device_info.vid = hexify(info.vid);
                device_info.pid = hexify(info.pid);
                device_info.unique_id = info.unique_id;
                device_info.device_path = info.unique_id;//the device unique_id is the USB port

                for(auto&& hs : hid_sensors)
                {
                    device_info.id = hs;
                    // LOG_INFO("Found HID device: " << std::string(device_info).c_str());
                    rv.push_back(device_info);
                }
            }
            return rv;
        }

        std::shared_ptr<hid_device> create_rshid_device(hid_device_info info)
        {
            auto devices = usb_enumerator::query_devices_info();
            for (auto&& usb_info : devices)
            {
                if(usb_info.unique_id != info.unique_id || usb_info.cls != RS2_USB_CLASS_HID)
                    continue;

                auto dev = usb_enumerator::create_usb_device(usb_info);
                return std::make_shared<rs_hid_device>(dev);
            }

            return nullptr;
        }

        rs_hid_device::rs_hid_device(rs_usb_device usb_device)
            : _usb_device(usb_device),
              _action_dispatcher(10)
        {
            _id_to_sensor[REPORT_ID_GYROMETER_3D] = gyro;
            _id_to_sensor[REPORT_ID_ACCELEROMETER_3D] = accel;
            _id_to_sensor[REPORT_ID_CUSTOM] = custom;
            _sensor_to_id[gyro] = REPORT_ID_GYROMETER_3D;
            _sensor_to_id[accel] = REPORT_ID_ACCELEROMETER_3D;
            _sensor_to_id[custom] = REPORT_ID_CUSTOM;
#ifdef __APPLE__
            hidapi_device_info *devices = hid_enumerate(0x8086, 0x0);
            if(devices)
            {
                _hidapi_device = hid_open(devices[0].vendor_id, devices[0].product_id, devices[0].serial_number);
                hidapi_PowerDevice(REPORT_ID_ACCELEROMETER_3D);
                hidapi_PowerDevice(REPORT_ID_GYROMETER_3D);
            }
#endif
            _action_dispatcher.start();
        }

        rs_hid_device::~rs_hid_device()
        {
#ifdef __APPLE__
            if( _hidapi_device )
                hid_close( _hidapi_device );
#endif
            _action_dispatcher.stop();
        }

        std::vector<hid_sensor> rs_hid_device::get_sensors()
        {
            std::vector<hid_sensor> sensors;

            for (auto& sensor : _hid_profiles)
                sensors.push_back({ sensor.sensor_name });

            return sensors;
        }

        void rs_hid_device::open(const std::vector<hid_profile>& hid_profiles)
        {
            for(auto&& p : hid_profiles)
            {
                set_feature_report(DEVICE_POWER_D0, _sensor_to_id[p.sensor_name], p.frequency);
            }
            _configured_profiles = hid_profiles;
        }

        void rs_hid_device::close()
        {
            set_feature_report(DEVICE_POWER_D4, REPORT_ID_ACCELEROMETER_3D);
            set_feature_report(DEVICE_POWER_D4, REPORT_ID_GYROMETER_3D);
        }

        void rs_hid_device::stop_capture()
        {
            _action_dispatcher.invoke_and_wait([this](dispatcher::cancellable_timer c)
            {
                if(!_running)
                    return;
#ifndef __APPLE__
                _request_callback->cancel();

                _queue.clear();

                for (auto&& r : _requests)
                    _messenger->cancel_request(r);

                _requests.clear();
#endif
                _handle_interrupts_thread->stop();
#ifndef __APPLE__
                _messenger.reset();
#endif
               _running = false;
            }, [this](){ return !_running; });
        }

        void rs_hid_device::start_capture(hid_callback callback)
        {
            _action_dispatcher.invoke_and_wait([this, callback](dispatcher::cancellable_timer c)
            {
                if(_running)
                    return;

                _callback = callback;
#ifndef __APPLE__
                auto in = get_hid_interface()->get_number();
                _messenger = _usb_device->open(in);
#endif
                _handle_interrupts_thread = std::make_shared<active_object<>>([this](dispatcher::cancellable_timer cancellable_timer)
                {
                    handle_interrupt();
                });

                _handle_interrupts_thread->start();

#ifndef __APPLE__
                _request_callback = std::make_shared<usb_request_callback>([&](platform::rs_usb_request r)
                    {
                        _action_dispatcher.invoke([this, r](dispatcher::cancellable_timer c)
                        {
                            if(!_running)
                                return;
                            int len = r->get_actual_length();
                            if(len == 38)
                            {
                                // 38-byte report (newer D455 firmware, 32-bit axes):
                                //   [0] report id, [1] ?, [2..9] timestamp (us),
                                //   [10..21] x,y,z int32, [22..29] timestamp again,
                                //   [30..35] x,y,z int16 (raw sensor units), [36..37] ?
                                // librealsense 2.50 expects the 32-byte report with int16 axes:
                                // accel 1 LSB = 0.001 g, gyro 1 LSB = 0.1 deg/s. Measured on two
                                // recorded walks (23 Oct; floor normal from depth, rotation from
                                // visual odometry, camera upright / upside down):
                                //   accel int32 = milli-g, sign as librealsense expects (3.7 deg
                                //   from the floor "up" with the camera upright) - copy;
                                //   gyro int32 = 0.0001 deg/s (|rate| matched VO only after / 1000;
                                //   the int16 at [30] is raw +-1000 deg/s, 3.28x too large in
                                //   0.1 deg/s units). Axes match the camera.
                                const uint8_t* b = r->get_buffer().data();
                                REALSENSE_HID_REPORT report{};
                                report.reportId = b[0];
                                report.unknown = b[1];
                                memcpy(&report.timeStamp, b + 2, sizeof(report.timeStamp));
                                int32_t v[3];
                                memcpy(v, b + 10, sizeof(v));
                                auto clamp16 = [](int64_t x) -> short {
                                    return static_cast<short>(std::max<int64_t>(-32768, std::min<int64_t>(32767, x)));
                                };
                                if (b[0] == REPORT_ID_ACCELEROMETER_3D)
                                {
                                    report.x = clamp16(v[0]);
                                    report.y = clamp16(v[1]);
                                    report.z = clamp16(v[2]);
                                }
                                else
                                {
                                    report.x = clamp16(v[0] / 1000);
                                    report.y = clamp16(v[1] / 1000);
                                    report.z = clamp16(v[2] / 1000);
                                }
                                _queue.enqueue(std::move(report));
                            }
                            else if(len >= sizeof(REALSENSE_HID_REPORT))
                            {
                                REALSENSE_HID_REPORT report;
                                memcpy(&report, r->get_buffer().data(), sizeof(REALSENSE_HID_REPORT));
                                _queue.enqueue(std::move(report));
                            }
                            else if(len > 0)
                            {
                                LOG_WARNING("HID interrupt received unexpected length: " << len
                                            << " expected " << sizeof(REALSENSE_HID_REPORT));
                            }
                            else
                            {
                                LOG_WARNING("HID interrupt received non-positive length: " << len);
                            }
                            auto sts = _messenger->submit_request(r);
                            if (sts != platform::RS2_USB_STATUS_SUCCESS)
                                LOG_ERROR("failed to submit HID interrupt request: " << sts);
                        });

                    });

                auto hid_ep = get_hid_endpoint();
                LOG_INFO("HID start_capture on ep " << (int)hid_ep->get_address() << " with " << USB_REQUEST_COUNT << " requests (buf=64)");
                _requests = std::vector<rs_usb_request>(USB_REQUEST_COUNT);
                for(auto&& r : _requests)
                {
                    r = _messenger->create_request(hid_ep);
                    r->set_buffer(std::vector<uint8_t>(64));
                    r->set_callback(_request_callback);
                }
#endif
                _running = true;
#ifndef __APPLE__
                for(auto&& r : _requests)
                {
                    auto sts = _messenger->submit_request(r);
                    LOG_INFO("HID start_capture submitted initial request sts=" << sts);
                }
#endif

            }, [this](){ return _running; });
        }

        void rs_hid_device::handle_interrupt()
        {
            REALSENSE_HID_REPORT report;
#ifdef __APPLE__
            hid_read(_hidapi_device, reinterpret_cast<unsigned char*>(&report), sizeof(REALSENSE_HID_REPORT));
            sensor_data data{};
            data.sensor = { _id_to_sensor[report.reportId] };

            hid_data hid{};
            hid.x = report.x;
            hid.y = report.y;
            hid.z = report.z;

            data.fo.pixels = &(hid.x);
            data.fo.metadata = &(report.timeStamp);
            data.fo.frame_size = sizeof(REALSENSE_HID_REPORT);
            data.fo.metadata_size = sizeof(report.timeStamp);

            _callback(data);
#else
            if(_queue.dequeue(&report, 10))
            {
                if(std::find_if(_configured_profiles.begin(), _configured_profiles.end(), [&](hid_profile& p)
                {
                    return _id_to_sensor[report.reportId].compare(p.sensor_name) == 0;

                }) != _configured_profiles.end())
                {
                    sensor_data data{};
                    data.sensor = { _id_to_sensor[report.reportId] };

                    hid_data hid{};
                    hid.x = report.x;
                    hid.y = report.y;
                    hid.z = report.z;

                    data.fo.pixels = &(hid.x);
                    data.fo.metadata = &(report.timeStamp);
                    data.fo.frame_size = sizeof(REALSENSE_HID_REPORT);
                    data.fo.metadata_size = sizeof(report.timeStamp);

                    _callback(data);
                }
            }
#endif
        }

        usb_status rs_hid_device::set_feature_report(unsigned char power, int report_id, int fps)
        {
            uint32_t transferred = 0;
            int value = (HID_REPORT_TYPE_FEATURE << 8) + report_id;
            auto hid_interface = get_hid_interface()->get_number();
            auto dev = _usb_device->open(hid_interface);

            if (!dev)
                return RS2_USB_STATUS_NO_DEVICE;

            FEATURE_REPORT featureReport{};
            featureReport.reportId = static_cast<unsigned char>(report_id);
            featureReport.connectionType = 0;
            featureReport.sensorState = 1;
            featureReport.power = power;
            featureReport.minReport = 1;
            auto calc_report_interval = [](int rep_id, int f) -> unsigned short {
                if (f == 1) return 10;       // 100 Hz -> 10 ms
                if (f == 2) return 5;        // 200 Hz -> 5 ms
                if (f == 3) return 4;        // 250 Hz -> 4 ms
                if (f == 4) return 2;        // 400 Hz -> 2 ms
                if (f >= 50) return static_cast<unsigned short>(1000 / f);
                return (rep_id == REPORT_ID_GYROMETER_3D) ? 5 : 10;
            };
            if (fps > 0)
                featureReport.report = calc_report_interval(report_id, fps);

            auto res = dev->control_transfer(USB_REQUEST_CODE_GET,
                HID_REQUEST_GET_REPORT,
                value,
                hid_interface,
                (uint8_t*) &featureReport,
                sizeof(featureReport),
                transferred,
                500);

            if(res != RS2_USB_STATUS_SUCCESS)
            {
                LOG_WARNING("control_transfer GET_REPORT (id " << report_id << ") returned " << res
                            << "; proceeding directly to SET_REPORT with power=" << (int)power << " fps=" << fps);
            }
            else
            {
                featureReport.power = power;
                if(fps > 0)
                    featureReport.report = calc_report_interval(report_id, fps);
            }

            // Always attempt SET_REPORT even if GET_REPORT was stalled by firmware:
            res = dev->control_transfer(USB_REQUEST_CODE_SET,
                HID_REQUEST_SET_REPORT,
                value,
                hid_interface,
                (uint8_t*) &featureReport,
                sizeof(featureReport),
                transferred,
                1000);

            if(res != RS2_USB_STATUS_SUCCESS)
            {
                LOG_WARNING("control_transfer of USB_REQUEST_CODE_SET (id " << report_id
                            << ") failed: " << res);
                return res;
            }

            LOG_INFO("HID Feature Report SET (id " << report_id << ") succeeded! power=" << (int)power
                     << " report=" << featureReport.report << " ms");
            return RS2_USB_STATUS_SUCCESS;
        }

#ifdef __APPLE__
        int rs_hid_device::hidapi_PowerDevice(unsigned char reportId)
        {
            if (_hidapi_device == NULL)
                return -1;

            REALSENSE_FEATURE_REPORT featureReport;
            int ret;

            memset(&featureReport,0, sizeof(featureReport));
            featureReport.reportId = reportId;
            // Reading feature report.
            ret = hid_get_feature_report(_hidapi_device, (unsigned char *)
                                         &featureReport ,sizeof(featureReport) );

            if (ret == -1) {
                LOG_ERROR("fail to read feature report from device");
                return ret;
            }

            // change report to power the device to D0

            featureReport.power = DEVICE_POWER_D0;

            // Write feature report back.
            ret = hid_send_feature_report(_hidapi_device, (unsigned char *) &featureReport, sizeof(featureReport));

            if (ret == -1) {
                LOG_ERROR("fail to write feature report from device");
                return ret;
            }

            ret = hid_get_feature_report(_hidapi_device, (unsigned char *) &featureReport ,sizeof(featureReport) );

            if (ret == -1) {
                LOG_ERROR("fail to read feature report from device");
                return ret;
            }

            if (featureReport.power == DEVICE_POWER_D0) {
                LOG_INFO("Device is powered up");
            } else {
                LOG_INFO("Device is powered off");
                return -1;
            }

            return 0;
        }
#endif

        rs_usb_interface rs_hid_device::get_hid_interface()
        {
            auto intfs = _usb_device->get_interfaces();

            auto it = std::find_if(intfs.begin(), intfs.end(),
                [](const rs_usb_interface& i) { return i->get_class() == RS2_USB_CLASS_HID; });

            if (it == intfs.end())
                throw std::runtime_error("can't find HID interface of device: " + _usb_device->get_info().id);

            return *it;
        }

        rs_usb_endpoint rs_hid_device::get_hid_endpoint()
        {
            auto hid_interface = get_hid_interface();

            auto ep = hid_interface->first_endpoint(RS2_USB_ENDPOINT_DIRECTION_READ, RS2_USB_ENDPOINT_INTERRUPT);
            if(!ep)
                throw std::runtime_error("can't find HID endpoint of device: " + _usb_device->get_info().id);

            return ep;
        }
    }
}
