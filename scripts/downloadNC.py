if __name__ == '__main__':
    from urllib2 import urlopen
    timeStamp = 1450558800
    remoteFilePath = "http://neocoweb.ucsd.edu/roms-3km/util/?func=get_data&ts="
    for x in range(0, 120):
        import os.path
        local_file_path = "../data/ocean/currents_nc/roms" + str(timeStamp) + ".nc"
        if not os.path.exists(local_file_path):
            print "downloading ", local_file_path
            f = open(local_file_path, 'wb')
            f.write(urlopen(remoteFilePath + str(timeStamp)).read())
            f.close()
        timeStamp += 21600
