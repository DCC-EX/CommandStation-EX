/*
 *  © 2024, Harald Barth. All rights reserved.
 *
 *  This file is part of DCC-EX
 *
 *  This is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  It is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.
 */
#ifndef TemplateForEnums
#define TemplateForEnums
template <class T>
inline T operator~(T a) {
  return (T) ~(int)a;
}
template <class T>
inline T operator|(T a, T b) {
  return (T)((int)a | (int)b);
}
template <class T>
inline T operator&(T a, T b) {
  return (T)((int)a & (int)b);
}
template <class T>
inline T operator^(T a, T b) {
  return (T)((int)a ^ (int)b);
}
#endif
