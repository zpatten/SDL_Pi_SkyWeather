#
#
# logging system from Project Curacao 
# filename: pclogger.py
# Version 1.0 10/04/13
#
# contains logging data 
#


CRITICAL=50
ERROR=40
WARNING=30
INFO=20
DEBUG=10
NOTSET=0


import sys
import time
# Check for user imports
try:
  import conflocal as config
except ImportError:
  import config

if (config.enable_MySQL_Logging == True):
	import MySQLdb as mdb


def log(level, source, message):
  if (config.enable_MySQL_Logging == True):	
    LOWESTDEBUG = 0
    if (level >= LOWESTDEBUG):
      try:
        con = mdb.connect(config.MySQL_Host, config.MySQL_Username, config.MySQL_Password, 'SkyWeather');
        cur = con.cursor()
        query = "INSERT INTO systemlog(TimeStamp, Level, Source, Message) VALUES(UTC_TIMESTAMP(), %i, '%s', '%s')" % (level, source, message)
        cur.execute(query)
        con.commit()
      except mdb.Error, e:
        print "Error %d: %s" % (e.args[0],e.args[1])
        con.rollback()
        #sys.exit(1)

      finally:
        cur.close()
        con.close()

        del cur
        del con
