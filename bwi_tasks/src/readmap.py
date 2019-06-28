from bwi_tools import getImageFileLocationFromMapFile as getImageFileLocation, loadMapFromFile
import os
import matplotlib.pyplot as plt
import numpy as np
np.set_printoptions(threshold=np.inf)
def slidingWindow(data):
        room_width = 80
        room_height = 110
        list_width = len(data[0])
        list_height = len(data)
        for i in range(200,310):
                for j in range(85,465):
                        data[j,i] = 5
        for i in range(530,620):
                for j in range(190,230):
                        data[j,i] = 5

        for i in range(575,645):
                for j in range(165,210):
                        data[j,i] = 5
        
        for i in range(810,950):
                for j in range(180,210):
                        data[j,i] = 5

        for i in range(915,950):
                for j in range(200,360):
                        data[j,i] = 5

        for i in range(660,800):
                for j in range(335,465):
                        data[j,i] = 5
        # for i in range(0,list_width - room_width+1):
        #         for j in range(0,list_height - room_height+1):
        #                 sub = data[j:j+room_height, i:i+room_width]
        #                 # print(sub)
        #                 sub = sub.flatten()
        #                 # print(sub)
        #                 if all(x == 0 or x==5 for x in sub) == True:
        #                         data[j:j+room_height, i:i+room_width] = 5
                        # print(list)
def main():    
        try:
            #data_folder = os.path.join("/home/users/NI2452/catkin_ws/src/","bwi_common/utexas_gdc/maps/simulation/3ne")
            data_folder = os.path.join("/home/brian/catkin_ws/src/","bwi_common/utexas_gdc/maps/simulation/3ne")
            yaml_file = os.path.join(data_folder, "doors_map.yaml")
        except KeyError:
            rospy.logfatal("map needs to be set to use the logical marker")
            return
        response = loadMapFromFile(yaml_file)
        map = response.map.data
        width = response.map.info.width
        height = response.map.info.height
        print(type(map), width, height, len(map))
        data = np.asarray(map)
        data = data.reshape(height, width)
        data[data==100] = 1
        data = data.astype(int)
        #room size, width = 80, height = 110
        slidingWindow(data)
        plt.imshow(data)
        plt.show()
        
        np.savetxt('test.out', data1, fmt='%d', delimiter=',')

if __name__ == '__main__':
        main()

