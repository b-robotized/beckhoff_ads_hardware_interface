// Copyright (c) 2025, b-robotized
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef BECKHOFF_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
#define BECKHOFF_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_EXPORT __attribute__((dllexport))
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_IMPORT __attribute__((dllimport))
#else
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_EXPORT __declspec(dllexport)
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_IMPORT __declspec(dllimport)
#endif
#ifdef TEMPLATES__ROS2_CONTROL__VISIBILITY_BUILDING_DLL
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC TEMPLATES__ROS2_CONTROL__VISIBILITY_EXPORT
#else
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC TEMPLATES__ROS2_CONTROL__VISIBILITY_IMPORT
#endif
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC_TYPE TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_LOCAL
#else
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_EXPORT __attribute__((visibility("default")))
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_IMPORT
#if __GNUC__ >= 4
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC __attribute__((visibility("default")))
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_LOCAL __attribute__((visibility("hidden")))
#else
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_LOCAL
#endif
#define TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC_TYPE
#endif

#endif  // BECKHOFF_HARDWARE_INTERFACE__VISIBILITY_CONTROL_H_
