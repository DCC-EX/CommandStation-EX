#!/usr/bin/env python3
#
#  Copyright 2022 Harald Barth
#
#  This converts serial logs with timestamps from the
#  Arduino IDE to LRC format.
#
#  This is free software: you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation, either version 3 of the License, or
#  (at your option) any later version.
#
#  It is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#
#  You should have received a copy of the GNU General Public License
#  along with CommandStation.  If not, see <https://www.gnu.org/licenses/>.

# Usage: log2lrc.py HH:MM:SS HH:MM:SS filename
#                   logtime  videotime
# The resulting timestamps will be adjusted by logtime - videotime.
# logtime > videotime

import sys
import datetime
import fileinput
import re

timediff = datetime.datetime.strptime('00:00:00','%H:%M:%S')

def convert(timestr):
    times=timestr.split('.',1) # remove fractions of second
    timestamp_obj=datetime.datetime.strptime(times[0],'%H:%M:%S')
    timestamp_obj=timestamp_obj-timediff       # calculate offset
    timestr='{0:%M:%S}'.format(timestamp_obj)
    timestr='%s.%s' % (timestr, times[1][0:2]) # add fractions of second, 2 digits
    return timestr

def main(argv):
    global timediff
    fromtime_str=sys.argv[1]
    fromtime_obj=datetime.datetime.strptime(fromtime_str,'%H:%M:%S')
    totime_str=sys.argv[2]
    totime_obj=datetime.datetime.strptime(totime_str,'%H:%M:%S')
    sys.argv=sys.argv[2:]
    

    timediff = fromtime_obj - totime_obj 

    for line in fileinput.input():
        line = re.split('\s+', line, 1)
        if (len(line) > 1 and len(line[1]) > 1):
            l = line[1].replace('\n','')
            l = l.replace('\r','')
            l = re.sub('^-> ','',l)
            print("[%s]%s" % (convert(line[0]),l))

if __name__ == "__main__":
    main(sys.argv)

