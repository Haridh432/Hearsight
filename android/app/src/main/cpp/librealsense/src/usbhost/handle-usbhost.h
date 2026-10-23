// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2015 Intel Corporation. All Rights Reserved.

#pragma once

#include "types.h"
#include "usbhost.h"
#include "interface-usbhost.h"

#include <vector>

namespace librealsense
{
    namespace platform
    {
        class handle_usbhost
        {
        public:
            handle_usbhost(::usb_device* device, std::shared_ptr<usb_interface_usbhost> interface) :
                _device(device)
            {
                claim_interface(interface);
                for(auto&& i : interface->get_associated_interfaces())
                {
                    claim_interface(i);
                }
            }

            ~handle_usbhost()
            {

//                for(auto&& i : _interfaces)
//                {
//                    usb_device_release_interface(_device, i->get_number());
//                }
            }

            void claim_interface(rs_usb_interface interface)
            {
                int r1 = usb_device_connect_kernel_driver(_device, interface->get_number(), false);
                if (r1 < 0) {
                    LOG_WARNING("usb_device_connect_kernel_driver(DISCONNECT) intf " << (int)interface->get_number()
                                << " returned " << r1 << " errno " << errno << " (" << strerror(errno) << ")");
                }
                int r2 = usb_device_claim_interface(_device, interface->get_number());
                if (r2 < 0) {
                    LOG_WARNING("usb_device_claim_interface intf " << (int)interface->get_number()
                                << " returned " << r2 << " errno " << errno << " (" << strerror(errno) << ")");
                } else {
                    LOG_INFO("usb_device_claim_interface intf " << (int)interface->get_number() << " claimed successfully");
                }
//                _interfaces.push_back(interface);
            }

            ::usb_device* _device;
//            std::vector<rs_usb_interface> _interfaces;
        };
    }
}
