/**
 * \file
 */

#ifndef GENICAM_API_H
#define GENICAM_API_H


#ifdef GENICAM_EXPORT
#define GENICAM_API __declspec(dllexport)
#else
#define GENICAM_API __declspec(dllimport)
#endif

#endif


