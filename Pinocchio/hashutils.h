/*  This file is part of the Pinocchio automatic rigging library.
    Copyright (C) 2007 Ilya Baran (ibaran@mit.edu)

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef HASHUTILS_H
#define HASHUTILS_H

#include "mathutils.h"

//#include <unordered_map>
#include <ext/hash_map>

#define _HASH_NAMESPACE __gnu_cxx

//using namespace _HASH_NAMESPACE;

namespace _HASH_NAMESPACE {
  template<class T1, class T2> struct hash<pair<T1, T2> > {
    size_t operator()(const pair<T1, T2> &p) const { return hash<T1>()(p.first) + 37 * hash<T2>()(p.second); }
  };

  template<class T> struct hash<T *> {
    size_t operator()(T *p) const { return (size_t)p; }
  };
}

//HASHUTILS_H
#endif
