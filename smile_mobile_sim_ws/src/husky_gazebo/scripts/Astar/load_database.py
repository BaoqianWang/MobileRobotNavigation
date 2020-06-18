import json

filename='path_database.txt'
with open(filename, 'r') as fd:
    database=json.load(fd)
    #print(json.load(fd))

print(database[0]['y'])
