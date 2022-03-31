// Copyright 2019 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROCK_RHINO_DATABASE__VISIBILITY_CONTROL_H_
#define ROCK_RHINO_DATABASE__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROCK_RHINO_DATABASE_EXPORT __attribute__ ((dllexport))
    #define ROCK_RHINO_DATABASE_IMPORT __attribute__ ((dllimport))
  #else
    #define ROCK_RHINO_DATABASE_EXPORT __declspec(dllexport)
    #define ROCK_RHINO_DATABASE_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROCK_RHINO_DATABASE_BUILDING_DLL
    #define ROCK_RHINO_DATABASE_PUBLIC ROCK_RHINO_DATABASE_EXPORT
  #else
    #define ROCK_RHINO_DATABASE_PUBLIC ROCK_RHINO_DATABASE_IMPORT
  #endif
  #define ROCK_RHINO_DATABASE_PUBLIC_TYPE ROCK_RHINO_DATABASE_PUBLIC
  #define ROCK_RHINO_DATABASE_LOCAL
#else
  #define ROCK_RHINO_DATABASE_EXPORT __attribute__ ((visibility("default")))
  #define ROCK_RHINO_DATABASE_IMPORT
  #if __GNUC__ >= 4
    #define ROCK_RHINO_DATABASE_PUBLIC __attribute__ ((visibility("default")))
    #define ROCK_RHINO_DATABASE_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROCK_RHINO_DATABASE_PUBLIC
    #define ROCK_RHINO_DATABASE_LOCAL
  #endif
  #define ROCK_RHINO_DATABASE_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ROCK_RHINO_DATABASE__VISIBILITY_CONTROL_H_
