#!/usr/bin/python
import threading
import datetime, time
import psycopg2
from psycopg2.extensions import ISOLATION_LEVEL_AUTOCOMMIT
from config import config

class DataManager:
    isConnected = False
    conn = None
    cur = None

    bufferCounter = 0
    bufferLimit = 5000
    timerInterval = 5.0
    sqlBuffer = []

    def connect(self,role):
        try:
            print('Connecting to the PostgreSQL database as {0} ...'.format(role))

            params = config(role)

            self.conn = psycopg2.connect(**params)
            self.conn.set_isolation_level(ISOLATION_LEVEL_AUTOCOMMIT)
            self.cur = self.conn.cursor()
        except (Exception, psycopg2.DatabaseError) as error:
            self.conn = None
            print(error)
        finally:
            if self.conn is not None:
                self.isConnected = True
                print('Database connection established.')
            else:
                self.isConnected = False
                print('Database not connected.')                

    def disConnect(self):
        if self.conn is not None:
            self.conn.close()
            self.isConnected = False
            print('Database connection closed.')

    def executeQuerry(self,query):
        if self.isConnected == True:
            self.cur.execute(query)

    def executeQuerryList(self, queryList):
        query = 'Begin;'
        if self.isConnected == True:
            for q in queryList:
                query += q
        query += 'Commit;'
        self.cur.execute(query)

    def initDatabase(self):
        databaseName='drillbotics'
        loggerUser='uis_logger'
        loggerPassword='Drillbotics'
        guestUser='guest'
        guestPassword='guest'

        self.connect('postgres')
        if self.isConnected == True:
            query=('SELECT exists(SELECT 1 from pg_catalog.pg_database where datname = \'{0}\')'.format(databaseName))
            self.executeQuerry(query)
            exists=self.cur.fetchone()[0]
            if exists == True:
                print('Database {0} exists.'.format(databaseName))
            else:
                query=( 'CREATE DATABASE {0}; ').format(databaseName)
                self.executeQuerry(query)
                print('Database {0} created.'.format(databaseName))                

            self.connect('admin')
            if self.isConnected == True:
                query=( 'DO '
                        '$body$ '
                        'BEGIN '
                        'IF NOT EXISTS (SELECT * FROM pg_catalog.pg_user WHERE usename=\'{1}\') THEN '
                        'CREATE ROLE {1} LOGIN PASSWORD \'{2}\'; '
                        'END IF; '
                        'IF NOT EXISTS (SELECT * FROM pg_catalog.pg_user WHERE usename=\'{3}\') THEN '
                        'CREATE ROLE {3} LOGIN PASSWORD \'{4}\'; '
                        'END IF; '
                        'END '
                        '$body$; '
                        'GRANT ALL PRIVILEGES ON DATABASE {0} TO {1}; '
                        'GRANT TRUNCATE ON ALL TABLES IN SCHEMA public TO {1}; '
                        'ALTER DEFAULT PRIVILEGES IN SCHEMA public GRANT TRUNCATE ON TABLES TO {1}; '
                        'ALTER DATABASE {0} OWNER TO {1}; '
                        'SET ROLE {1}; '
                        'GRANT CONNECT ON DATABASE {0} TO {3}; '
                        'GRANT USAGE ON SCHEMA public TO {3}; '
                        'ALTER DEFAULT PRIVILEGES IN SCHEMA public GRANT SELECT ON TABLES TO {3}; '
                        'ALTER DEFAULT PRIVILEGES IN SCHEMA public GRANT SELECT ON SEQUENCES TO {3};'.format(databaseName,loggerUser,loggerPassword,guestUser,guestPassword))
                self.executeQuerry(query)
            else:
                print('Database not connected.')                
        else:
            print('Database not connected.')                

    def dropDatabase(self):
        databaseName='drillbotics'

        self.connect('postgres')
        if self.isConnected == True:
            query=( 'SELECT pg_terminate_backend(pg_stat_activity.pid) FROM pg_stat_activity '
                    'WHERE pg_stat_activity.datname = \'{0}\' AND pid <> pg_backend_pid();'.format(databaseName))
            self.executeQuerry(query)
            query= ('DROP DATABASE IF EXISTS {0};'.format(databaseName))
            self.executeQuerry(query)
            print('Database {0} dropped.'.format(databaseName))
        else:
            print('Database not connected.')                

    def dropTable(self,tableName):
        query=('DROP TABLE IF EXISTS "t_{0}" CASCADE;'.format(tableName))
        self.executeQuerry(query)

    def createTable(self,tableName):
        query=('CREATE TABLE IF NOT EXISTS "t_{0}" (time numeric not null,data numeric not null,id bigserial primary key);'
               'CREATE UNIQUE INDEX IF NOT EXISTS "t_{0}_idx" ON "t_{0}" (time);').format(tableName)
        if tableName == "visitors":
            query=('CREATE TABLE IF NOT EXISTS "visitors" (time numeric primary key,datetime text not null,ip text);')

        self.executeQuerry(query)

    def fetchList(self):
        query=('SELECT row_number() OVER (ORDER BY table_name) as recid, replace(table_name, \'t_\', \'\') AS sensorname '
               'FROM information_schema.tables WHERE table_schema=\'public\' AND table_type=\'BASE TABLE\' AND table_name '
               'LIKE \'t\\_%\' AND table_name <> \'t_PH-\' OR table_name=\'eventlog\' ORDER BY recid')
        self.executeQuerry(query)
        
    def pushToDatabase(self,sqlBufferClone,bufferCounterClone):
        startTime = time.time()
        self.executeQuerryList(sqlBufferClone)
        elapsed = time.time()-startTime

        print('[{0}]: {1} records inserted in {2} ms.\r'.format(datetime.datetime.now().strftime('Y-%m-%d %H:%M:%S'),bufferCounterClone, elapsed*1000))
    
    def pushIntoSqlBuffer(self,tableName,value):
        timeStamp = (datetime.datetime.utcnow() - datetime.datetime(1970,1,1)).total_seconds()
        header = 'INSERT INTO \"t_{0}\" (time, data) VALUES'.format(tableName)
            
        found = False
        i = 0
        while i< len(self.sqlBuffer):
            sql = self.sqlBuffer[i]
            if header in sql:
                self.sqlBuffer[i] = '{0}, ({1},{2});'.format(sql[:-1],timeStamp,value)
                found = True
                break
            i+=1

        if found == False:
            sql = '{0} ({1},{2});'.format(header, timeStamp, value)
            self.sqlBuffer.append(sql)

        self.bufferCounter += 1
        if self.bufferCounter >= self.bufferLimit:
            sqlBufferClone = list(self.sqlBuffer)
            bufferCounterClone = self.bufferCounter
            self.sqlBuffer = []
            self.bufferCounter = 0

            dataPusher = threading.Thread(name='dataPusher', target=self.pushToDatabase, args=(sqlBufferClone,bufferCounterClone))
            dataPusher.start()