import os
import sys
import math
import statistics
import time

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# function used to read gps/acomms data from in-water 
# tests perfomed on 10-26-15 and 10-27-15
def main():

    # Use the mode variable to define what file to read
    # MODE = 26 for gps-acomms-2015-10-26.txt"
    # MODE = 27 for gps-acomms-2015-10-27.txt"
    # MODE = 28 for gps-acomms-2015-10-26_27_combo.txt
    MODE = 27
    txt_file_name = get_full_file_name(MODE)

    try:
        txt_file = open(txt_file_name, 'r')
    except FileNotFoundError:
        print('Text file not found in working directory of gps_acomms_read_exhaust.py')
        sys.exit()

    veh_pos = [] #vehicle
    ts_pos_rx = [] #topside 
    for message in message_iterator(txt_file):
        msg_header = message[0]

        # indicates an error message
        if msg_header == '$CAMSG':
            #re-use previous veh_pos as nearest estimate
            last_pos = veh_pos[-1]
            last_pos[0] = 0
            veh_pos.append(last_pos)

            #signal strength
            sig_str = float(message[3])

        # indicates a successful message
        elif msg_header == '$CARXD':
            # use this to extract byte info
            gg = message[5]

            # get lat, long, & depth, and reverse their endianness
            msg_lat_rev_endian = gg[6:8] + gg[4:6] + gg[2:4] 
            msg_long_rev_endian = gg[12:14] + gg[10:12] + gg[8:10]
            msg_depth_rev_endian_encoded = gg[26:28] + gg[24:26]

            # convert from hex to dec
            msg_lat = int(msg_lat_rev_endian, 16)
            msg_long = int(msg_long_rev_endian, 16)
            msg_depth_encoded = int(msg_depth_rev_endian_encoded, 16)

            # decode depth
            depth = msg_depth_encoded & int('1FFF', 16)

            if depth <= 1000:
                vz = 0.1 * depth 
            elif (depth > 1000 and depth <= 1500):
                vz = 100 + 0.2*(depth - 1000)
            elif (depth > 1500 and depth <= 3100):
                vz = 200 + 0.5*(depth - 1500)
            elif (depth > 3100 and depth <= 8100):
                vz = 1000 + 1.0*(depth - 3100)
            else:
                vz = 6000

            # decode int decimal degrees
            # 8388607 (dec) = 0x07FFFFF (hex) and is used to define
            # the LSB resolution
            msg_lat = msg_lat*(180/8388607)
            msg_long = msg_long*(180/8388607)
            [vx, vy, zone_number, zone_letter] = from_latlon(msg_lat, msg_long)

            veh_pos.append([1, vx, vy, vz])

            # signal strength
            sig_str = float(message[6])

        # Retrieve latest GPS position for reference. This position 
        # represents the nearest location at which the topside vehicle 
        # receives the reply regardless whether the reply is a 
        # success ($CARDX) or failure ($CAMSG).
        for line in txt_file:
            if "$GPGGA" in line:
                gps_line = line.split(',')
                break

        # assert that a $GPGGA message has been acquired
        assert(gps_line[0] == "$GPGGA")
        time_stamp = int(gps_line[1])
        
        #Just for reference, use the following to decode 9-11 bytes
        #containing time information in a received CCL message
#        vSec = bitand(timeStamp,15)*4;
#        vMin = 60*(bitand(bitshift(timeStamp,4),hex2dec('3F')));
#        vHour = 60*60*(bitand(bitshift(timeStamp,10),hex2dec('1F')));
#        seconds = (4*(time_stamp & 15))
#        minutes = (60*((time_stamp << 4) & 63))
#        hours = (60*60*((time_stamp << 10) & 31)


        # convert from decimal-decimal to decimal-degrees
        lat_dd = decdec2decdeg(float(gps_line[2]))
        long_dd = decdec2decdeg(float(gps_line[4]))

        # convert from decimal-degrees to utm
        [x, y, zone_number, zone_letter] = from_latlon(lat_dd, long_dd)

        # fix quality flag - not currently used
        fix_q = int(gps_line[6])

        pos_vec = [time_stamp, x, y, sig_str]

        # store topside position upon receiving reply
        ts_pos_rx.append(pos_vec)

    txt_file.close()

    if MODE == 26: # filter 10-26 data
        # filter out positions based on ts_pos_rx
        filter_out(ts_pos_rx, veh_pos, 4.59e6, 4.603e6)
        # filter out positions based on veh_pos
        filter_out(veh_pos, ts_pos_rx, 4.6005e6, 4.6025e6)
    elif MODE == 27: # filter 10-27 data
        # filter out positions based on ts_pos_rx
        filter_out(ts_pos_rx, veh_pos, 4.5841e6, 4.5862e6)
        # filter out positions based on veh_pos
        filter_out(veh_pos, ts_pos_rx, 4.584e6, 4.5865e6)
    elif MODE == 28: # filter combined data
        # filter out positions based on ts_pos_rx
        filter_out(ts_pos_rx, veh_pos, 4.5841e6, 4.603e6)
        # filter out positions based on veh_pos
        filter_out(veh_pos, ts_pos_rx, 4.584e6, 4.6025e6)

    #open files for writing vehicle position information
    ts_pos_file = open("ts_pos.csv", "w")
    veh_pos_file = open("veh_pos.csv", "w")
    
    ts_pos_file.write("time stamp, x, y, signal strength\n")
    veh_pos_file.write("success, x, y, z\n")

    ts_pos1_rx_aver = statistics.mean([pos[1] for pos in ts_pos_rx])
    ts_pos2_rx_aver = statistics.mean([pos[2] for pos in ts_pos_rx])

    for pos in ts_pos_rx:
        pos[1] = pos[1] - ts_pos1_rx_aver
        pos[2] = pos[2] - ts_pos2_rx_aver
        ts_pos_file.write("%s\n" % (str(pos))[1:-1])

    veh_pos1_aver = statistics.mean([pos[1] for pos in veh_pos])
    veh_pos2_aver = statistics.mean([pos[2] for pos in veh_pos])

    for pos in veh_pos:
        pos[1] = pos[1] - veh_pos1_aver
        pos[2] = pos[2] - veh_pos2_aver
        veh_pos_file.write("%s\n" % (str(pos))[1:-1])

    veh_pos_file.close()
    ts_pos_file.close()
    
    plot_vehicle_data(veh_pos, ts_pos_rx)
    

    
def plot_vehicle_data(veh_pos, ts_pos_rx):
    fig, ax = plt.subplots()

    x = np.arange(0, 0.5, 0.01)
    line, = ax.plot(x, np.sin(x))
    
    def animate(i):
        line.set_ydata(np.sin(x + 10.0))  # update the data
        return line,

    # Init only required for blitting to give a clean slate.
    def init():
        line.set_ydata(np.ma.array(x, mask=True))
        return line,

    ani = animation.FuncAnimation(fig, animate, np.arange(1, 200), init_func=init,
                                  interval=25, blit=True)
    
    plt.show()


    
def update_line(num, data, line):
    line.set_data(data[..., :num])
    return line,

# Generator used to iterate through text file and return messages
# @param text file - open text file to parse
# @return message array with sig_str appended to the end
def message_iterator(text_file):
    with text_file as myfile:
        # find messages that appear after we find a header with "$CACYC" followed by "$CADQF"
        for line in myfile:
            # read until we find a message with header "$CACYC"
            if "$CACYC" in line:
                # read until we find a message with header "$CADQF"
                for line in myfile:
                    if "$CADQF" in line:
                        break
                # check to make sure the line has "$CADQF" and isn't just the last line
                if "CADQF" in line:
                    #Get this so we can include it in the message
                    sig_str = (line.split(','))[1]
                    # read the line after the one with "$CADQF" and return it as an array
                    # delimited by commas
                    message = myfile.readline().split(',')
                    # Add sig_str to the end of the message
                    message.append(sig_str)
                    yield message


# Return the file name here based on the MODE value defined in main()
# This function assumes the python file is in same directory as the text files
def get_full_file_name(mode):

    pwd = os.getcwd() 
    return {
        26: pwd + os.path.sep + "gps-acomms-2015-10-26.txt",
        27: pwd + os.path.sep + "gps-acomms-2015-10-27.txt", 
        28: pwd + os.path.sep + "gps-acomms-2015-10-26_27_combo.txt",
    }.get(mode, "gps-acomms-2015-10-26.txt")  #return 26 if a mode other than 26, 27, or 28 is passed


# Filter out elements from filter_from_array whose values at index 2 are outside of
# min_thresh and max_thresh. Take the indices of removed elements from filter_from_list 
# and remove the elements of follower_list at those same indices.  
def filter_out(filter_from_list, follower_list, min_thresh, max_thresh):
    remove_indices = [i for (i, pos) in enumerate(filter_from_list) if check_val(pos[2], min_thresh, max_thresh)]
    removed_elems = 0
    for i in remove_indices:
        del(filter_from_list[i - removed_elems])
        del(follower_list[i - removed_elems])
        removed_elems += 1

    #print(veh_pos)
    #print(ts_pos_rx)

def check_val(element, min_thresh, max_thresh):
    if (element < min_thresh or element > max_thresh):
        return True
    else:
        return False

def decdec2decdeg(decdec_val):
    deg = math.floor(decdec_val / 100)
    dec = ((decdec_val / 100) - deg)/0.60
    return (deg+dec)


# https://pypi.python.org/pypi/utm
class OutOfRangeError(ValueError):
    pass

# https://pypi.python.org/pypi/utm
def from_latlon(latitude, longitude, force_zone_number=None):
    """This function convert Latitude and Longitude to UTM coordinate

        Parameters
        ----------
        latitude: float
            Latitude between 80 deg S and 84 deg N, e.g. (-80.0 to 84.0)

        longitude: float
            Longitude between 180 deg W and 180 deg E, e.g. (-180.0 to 180.0).

        force_zone number: int
            Zone Number is represented with global map numbers of an UTM Zone
            Numbers Map. You may force conversion including one UTM Zone Number.
            More information see utmzones [1]_

       .. _[1]: http://www.jaworski.ca/utmzones.htm
    """
    __all__ = ['to_latlon', 'from_latlon']

    K0 = 0.9996

    E = 0.00669438
    E2 = E * E
    E3 = E2 * E
    E_P2 = E / (1.0 - E)

    SQRT_E = math.sqrt(1 - E)
    _E = (1 - SQRT_E) / (1 + SQRT_E)
    _E2 = _E * _E
    _E3 = _E2 * _E
    _E4 = _E3 * _E
    _E5 = _E4 * _E

    M1 = (1 - E / 4 - 3 * E2 / 64 - 5 * E3 / 256)
    M2 = (3 * E / 8 + 3 * E2 / 32 + 45 * E3 / 1024)
    M3 = (15 * E2 / 256 + 45 * E3 / 1024)
    M4 = (35 * E3 / 3072)

    P2 = (3. / 2 * _E - 27. / 32 * _E3 + 269. / 512 * _E5)
    P3 = (21. / 16 * _E2 - 55. / 32 * _E4)
    P4 = (151. / 96 * _E3 - 417. / 128 * _E5)
    P5 = (1097. / 512 * _E4)

    R = 6378137

#    if not -80.0 <= latitude <= 84.0:
#        print('Warning: latitude {val} out of range (must be between 80 deg S and 84 deg N)'.format(val=round(latitude,2)))
#    if not -180.0 <= longitude <= 180.0:
#        print('Warning: longitude {val} out of range (must be between 180 deg W and 180 deg E)'.format(val=round(longitude,2)))

    lat_rad = math.radians(latitude)
    lat_sin = math.sin(lat_rad)
    lat_cos = math.cos(lat_rad)

    lat_tan = lat_sin / lat_cos
    lat_tan2 = lat_tan * lat_tan
    lat_tan4 = lat_tan2 * lat_tan2

    if force_zone_number is None:
        zone_number = latlon_to_zone_number(latitude, longitude)
    else:
        zone_number = force_zone_number

    zone_letter = latitude_to_zone_letter(latitude)

    lon_rad = math.radians(longitude)
    central_lon = zone_number_to_central_longitude(zone_number)
    central_lon_rad = math.radians(central_lon)

    n = R / math.sqrt(1 - E * lat_sin**2)
    c = E_P2 * lat_cos**2

    a = lat_cos * (lon_rad - central_lon_rad)
    a2 = a * a
    a3 = a2 * a
    a4 = a3 * a
    a5 = a4 * a
    a6 = a5 * a

    m = R * (M1 * lat_rad -
             M2 * math.sin(2 * lat_rad) +
             M3 * math.sin(4 * lat_rad) -
             M4 * math.sin(6 * lat_rad))

    easting = K0 * n * (a +
                        a3 / 6 * (1 - lat_tan2 + c) +
                        a5 / 120 * (5 - 18 * lat_tan2 + lat_tan4 + 72 * c - 58 * E_P2)) + 500000

    northing = K0 * (m + n * lat_tan * (a2 / 2 +
                                        a4 / 24 * (5 - lat_tan2 + 9 * c + 4 * c**2) +
                                        a6 / 720 * (61 - 58 * lat_tan2 + lat_tan4 + 600 * c - 330 * E_P2)))

    if latitude < 0:
        northing += 10000000

    return easting, northing, zone_number, zone_letter

# https://pypi.python.org/pypi/utm
def latitude_to_zone_letter(latitude):
    ZONE_LETTERS = "CDEFGHJKLMNPQRSTUVWXX"
    if -80 <= latitude <= 84:
        return ZONE_LETTERS[int(latitude + 80) >> 3]
    else:
        return None

# https://pypi.python.org/pypi/utm
def latlon_to_zone_number(latitude, longitude):
    if 56 <= latitude < 64 and 3 <= longitude < 12:
        return 32

    if 72 <= latitude <= 84 and longitude >= 0:
        if longitude <= 9:
            return 31
        elif longitude <= 21:
            return 33
        elif longitude <= 33:
            return 35
        elif longitude <= 42:
            return 37

    return int((longitude + 180) / 6) + 1

# https://pypi.python.org/pypi/utm
def zone_number_to_central_longitude(zone_number):
    return (zone_number - 1) * 6 - 180 + 3

if __name__ == "__main__":
    main()