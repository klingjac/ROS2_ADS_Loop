# from ADCS.lib.RTC_Driver import RV_8803
import datetime
import time
from dateutil import tz

def resetRTC(rtc):
    "[{seconds}, {minutes}, {hours}, {weekday (1-7, 1 is Sunday)}, {day}, {month}, {year}]"
    rtc.setFullTime([0,0,0,7,13,4,2024])

def dt_utc2est(utc_datetime):
    est_datetime = utc_datetime + datetime.timedelta(hours = -4)
    return est_datetime

def gps_utc2utc(utc_time_str):
    # Parse the UTC time and date strings
    utc_datetime = datetime.datetime.strptime(f"{utc_time_str}", "%H%M%S.%f")
    est_datetime = dt_utc2est(utc_datetime)

    # Extract seconds, minutes, hours, and date
    seconds = utc_datetime.second
    minutes = utc_datetime.minute
    hours = utc_datetime.hour

    return [seconds, minutes, hours]

# -- Legacy Testing --
def test():
    rtc = RV_8803()
    now = gps_utc2est("065959.546")
    # rtc.setTime(now)
    # rtc.setCalendar([13, 4, 2024])
    # for i in range(2):
    #     print(rtc.getFullTime())
    #     time.sleep(0.2)
    # resetRTC(rtc)

print(gps_utc2utc("123647.876"))