/*
 *  Queue.h
 * 
 *  This file is part of CommandStation-EX.
 *
 *  CommandStation-EX is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  CommandStation-EX is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with CommandStation-EX.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef UTILS_QUEUE_H_
#define UTILS_QUEUE_H_

#include <Arduino.h>

template<class T, int S>
class Queue {
private:
  int _front, _back, _count;
  T _data[S+1];
  int _maxitems;
public:
  Queue() { 
    _front = 0;
    _back = 0;
    _count = 0;
    _maxitems = S;   
  }
  ~Queue() {
    // delete[] _data;  
  }
  inline int count();
  inline int front();
  inline int back();
  void push(const T &item);
  T peek();
  T pop();
  void clear();
};

template<class T, int S>
inline int Queue<T, S>::count() 
{
  return _count;
}

template<class T, int S>
inline int Queue<T, S>::front() 
{
  return _front;
}

template<class T, int S>
inline int Queue<T, S>::back() 
{   
  return _back;
}

template<class T, int S>
void Queue<T, S>::push(const T &item)
{
  noInterrupts();
  if(_count < _maxitems) { // Drops out when full
    _data[_back++]=item;
    ++_count;
    // Check wrap around
    if (_back > _maxitems)
    _back -= (_maxitems + 1);
  }
  interrupts();
}

template<class T, int S>
T Queue<T, S>::pop() {
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

template<class T, int S>
T Queue<T, S>::peek() {
  if(_count <= 0) return T(); // Returns empty
  else return _data[_front];
}

template<class T, int S>
void Queue<T, S>::clear() 
{
  _front = _back;
  _count = 0;
}

#endif  // UTILS_QUEUE_H_
