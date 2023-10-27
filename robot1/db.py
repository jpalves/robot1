# connect_db.py
# import os
import sqlite3
from os.path import isfile, getsize
from ament_index_python.packages import get_package_share_directory
dir = get_package_share_directory('robot1')

class Connect(BaseException):

    def __init__(self, db_name):
        try:
            # conectando...
            self.conn = sqlite3.connect(db_name)
            self.cursor = self.conn.cursor()
            # imprimindo nome do banco
            print("Base de dados:", db_name)
            # lendo a versão do SQLite
            self.cursor.execute('SELECT SQLITE_VERSION()')
            self.data = self.cursor.fetchone()
            # imprimindo a versão do SQLite
            print("SQLite version: %s" % self.data)
        except sqlite3.Error as e:
            print("Erro ao abrir base de dados.")
            #return False

    def close_db(self):
        if self.conn:
            self.conn.close()
            print("Conexão fechada.")
            
    def commit_db(self):
        return self.conn.commit()
        
class Db(BaseException):

    tb_name = 'machines'

    def __init__(self):
        print(dir + '/resource/machines.db')
        self.db = Connect(dir + '/resource/machines.db')
        #self.tb_name

    def close_connection(self):
        self.db.close_db()
        
    def isSQLite3(self, filename):
        if not isfile(filename):
            return False
        if getsize(filename)< 100: # SQLite database file header is 100 bytes
            return False

        with open(filename, 'rb') as fd:
           header = fd.read(100)

        return True #header[:16] == 'SQLite format 3\x00'
    
    # create_schema
    def criar_schema(self, schema_name= dir + '/resource/machines_esquema.sql'):
        if self.isSQLite3('machines.db'):
            return
            
        print("Criando tabela %s ..." % self.tb_name)
       
        try:
            with open(schema_name, 'rt') as f:
                schema = f.read()
                self.db.cursor.executescript(schema)
        except sqlite3.Error as e:
            print(sqlite3.Error)
            #print("Aviso: A tabela %s já existe." % self.tb_name)
            return False

        print("Tabela %s criada com sucesso." % self.tb_name)
    
    def inserir(self, m_id, x, y, yaw, tag):
        try:
            self.db.cursor.execute("INSERT INTO machines (id , x, y, yaw, tag) VALUES (?,?,?,?,?)",(m_id,x,y,yaw,tag,))
            # gravando no bd
            self.db.commit_db()
            print("inserido com sucesso.")
        except sqlite3.IntegrityError as e:
            print("o ID é único")
            
    def ler(self):
        sql = 'SELECT * FROM machines ORDER BY id'
        try:
            r = self.db.cursor.execute(sql)
            return r.fetchall()
        except:
            print('ainda não está criada')
       	      
    def apagar(self, id):
        try:
            self.db.cursor.execute("DELETE FROM machines WHERE id = ?", (id,))
            self.db.commit_db()
        except:
            print('não existe')  


