import os
import oss2
import pymongo
def get_oss_mongo():
    access_key_id = os.getenv('OSS_TEST_ACCESS_KEY_ID', 'LTAI4GJDtEd1QXeUPZrNA4Yc')
    access_key_secret = os.getenv('OSS_TEST_ACCESS_KEY_SECRET', 'rxWAZnXNhiZ8nemuvshvKxceYmUCzP')
#    endpoint = os.getenv('OSS_TEST_ENDPOINT', 'oss-cn-beijing-internal.aliyuncs.com')
    endpoint = os.getenv('OSS_TEST_ENDPOINT', 'oss-cn-beijing.aliyuncs.com')
    bucket_name='ride-v'
    bucket = oss2.Bucket(oss2.Auth(access_key_id, access_key_secret), endpoint, bucket_name)
    url="mongodb://root:Lance1809@dds-2ze6c59cc1a69bf41591-pub.mongodb.rds.aliyuncs.com:3717,dds-2ze6c59cc1a69bf42307-pub.mongodb.rds.aliyuncs.com:3717/admin?replicaSet=mgset-32634159"
#    url="mongodb://root:Lance1809@dds-2ze6c59cc1a69bf41.mongodb.rds.aliyuncs.com:3717,dds-2ze6c59cc1a69bf42.mongodb.rds.aliyuncs.com:3717/admin?replicaSet=mgset-32634159"
    myclient = pymongo.MongoClient(url)
    return [bucket, myclient]
