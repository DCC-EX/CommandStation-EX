/*
 *  © 2021, Gregor Baues, All rights reserved.
 *  
 *  This file is part of DCC-EX/CommandStation-EX
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
 * 
 */

#ifndef _Queue_h_
#define _Queue_h_

#include <Arduino.h>

template<class T>
class Queue {
  
  private:
    int _front, _back, _count;
    T *_data;
    int _maxitems;

  public:
    Queue(int maxitems = 256) { 
      _front = 0;
      _back = 0;
      _count = 0;
      _maxitems = maxitems;
      _data = new T[maxitems + 1];   
    }
    ~Queue() {
      delete[] _data;  
    }
    inline int count();
    inline int front();
    inline int back();
    void push(const T &item);
    T peek();
    T pop();
    void clear();
};

template<class T>
inline int Queue<T>::count() 
{
  return _count;
}

template<class T>
inline int Queue<T>::front() 
{
  return _front;
}

template<class T>
inline int Queue<T>::back() 
{
  return _back;
}

template<class T>
void Queue<T>::push(const T &item)
{
  if(_count < _maxitems) { // Drops out when full
    _data[_back++]=item;
    ++_count;
    // Check wrap around
    if (_back > _maxitems)
      _back -= (_maxitems + 1);
  }
}

template<class T>
T Queue<T>::pop() {
  if(_count <= 0) return T(); // Returns empty
  else {
    T result = _data[_front];
    _front++;
    --_count;
    // Check wrap around
    if (_front > _maxitems) 
      _front -= (_maxitems + 1);
    return result; 
  }
}

template<class T>
T Queue<T>::peek() {
  if(_count <= 0) return T(); // Returns empty
  else return _data[_front];
}

template<class T>
void Queue<T>::clear() 
{
  _front = _back;
  _count = 0;
}

#endif // _Queue_h_










