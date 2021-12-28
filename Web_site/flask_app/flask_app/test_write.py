import time
import json as simplejson

alist0=[43.57076, 1.46604]
alist1=[44.57076, 1.46605]
alist2=[45.57076, 1.46605]
alist3=[46.57076, 1.46605]

f = open('fire_coord.txt', 'w')
simplejson.dump(alist0, f)
f.write("\n")
f.close()

time.sleep(10)

f = open('fire_coord.txt', 'w')
simplejson.dump(alist1, f)
f.write("\n")
f.close()

time.sleep(10)

f = open('fire_coord.txt', 'w')
simplejson.dump(alist2, f)
f.write("\n")
f.close()

time.sleep(10)

f = open('fire_coord.txt', 'w')
simplejson.dump(alist3, f)
f.write("\n")
f.close()