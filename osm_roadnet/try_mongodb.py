import pymongo
url="mongodb://root:Lance1809@dds-2ze6c59cc1a69bf41591-pub.mongodb.rds.aliyuncs.com:3717,dds-2ze6c59cc1a69bf42307-pub.mongodb.rds.aliyuncs.com:3717/admin?replicaSet=mgset-32634159"
myclient = pymongo.MongoClient(url)
print(myclient)
mydb = myclient["mydatabase"]
mycol = mydb["customers"]
myquery = { "address": "Park Lane 38" }
mydoc = mycol.find(myquery,{"_id":0,"name":1})
for x in mydoc:
    print(x)
