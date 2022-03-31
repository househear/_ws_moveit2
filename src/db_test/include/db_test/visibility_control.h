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

#ifndef DB_TEST__VISIBILITY_CONTROL_H_
#define DB_TEST__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define DB_TEST_EXPORT __attribute__ ((dllexport))
    #define DB_TEST_IMPORT __attribute__ ((dllimport))
  #else
    #define DB_TEST_EXPORT __declspec(dllexport)
    #define DB_TEST_IMPORT __declspec(dllimport)
  #endif
  #ifdef DB_TEST_BUILDING_DLL
    #define DB_TEST_PUBLIC DB_TEST_EXPORT
  #else
    #define DB_TEST_PUBLIC DB_TEST_IMPORT
  #endif
  #define DB_TEST_PUBLIC_TYPE DB_TEST_PUBLIC
  #define DB_TEST_LOCAL
#else
  #define DB_TEST_EXPORT __attribute__ ((visibility("default")))
  #define DB_TEST_IMPORT
  #if __GNUC__ >= 4
    #define DB_TEST_PUBLIC __attribute__ ((visibility("default")))
    #define DB_TEST_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define DB_TEST_PUBLIC
    #define DB_TEST_LOCAL
  #endif
  #define DB_TEST_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // DB_TEST__VISIBILITY_CONTROL_H_
