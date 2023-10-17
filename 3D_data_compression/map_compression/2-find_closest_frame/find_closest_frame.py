import pandas as pd
import numpy as np
import sys


# Usage 
# python find_closest_frame.py [dataset folder path] [day1 path (reference day) ] [day2 path (day to be compressed)] [x] [y] [z]

def remove_extension(integer):
    return integer[:-4]


#Load command line arguments 

data_path = sys.argv[1]
ref_path = data_path + '/' + sys.argv[2] 
com_path = data_path + '/' + sys.argv[3]
x = int(sys.argv[4])
y = int(sys.argv[5])
z = int(sys.argv[6])
output_file = com_path+ '/association.txt'

ref_pose_file = ref_path + '/pose.txt'
com_pose_file = com_path + '/pose.txt'

# ------ load position of to be compressed frames

################ used whwen there is .pcd in the name of frame in the pose file 
compress_df = pd.read_csv(com_pose_file, sep=",", header=None, float_precision='round_trip')
# compress_df[compress_df.columns[0]] = compress_df[compress_df.columns[0]].apply(remove_extension)
# compress_df[compress_df.columns[0]] = compress_df[compress_df.columns[0]].astype('float64')
com_frame_names = compress_df[compress_df.columns[0]].to_numpy()
com_location_data = compress_df[compress_df.columns[x:z+1]].to_numpy()
com_location_data= np.transpose(com_location_data)


################ used whwen there is no .pcd in the name of frame in the pose file 
# com_frame_names = np.loadtxt(com_pose_file, delimiter=',', usecols=(0), unpack=True)
# com_location_data  = np.loadtxt(com_pose_file, delimiter=',', usecols=(x,y,z), unpack=True)

com_location_data = np.reshape(com_location_data, (3, -1,1))
ones = -1*np.ones((3,com_location_data.shape[1],1))
com_location_data = np.append(com_location_data, ones, 2)
print(com_location_data.shape)

# -------- load position of ref frames

################ used whwen there is .pcd in the name of frame in the pose file 

reference_df = pd.read_csv(ref_pose_file, sep=",", header=None, float_precision='round_trip')
# reference_df[reference_df.columns[0]] = reference_df[reference_df.columns[0]].apply(remove_extension)
# reference_df[reference_df.columns[0]] = reference_df[reference_df.columns[0]].astype('float64')
ref_frame_names = reference_df[reference_df.columns[0]].to_numpy()
ref_location_data = reference_df[reference_df.columns[x:z+1]].to_numpy()
ref_location_data = np.transpose(ref_location_data)


################ used whwen there is no .pcd in the name of frame in the pose file 

# ref_frame_names = np.loadtxt(ref_pose_file, delimiter=',', usecols=(0), unpack=True)
# ref_location_data  = np.loadtxt(ref_pose_file, delimiter=',', usecols=(x,y,z), unpack=True)

ref_frame_names_dict = dict(enumerate(ref_frame_names))
ref_location_data = np.reshape(ref_location_data, (3,1,-1))
ones = np.ones((3,1, ref_location_data.shape[2]))
ref_location_data = np.append(ones,ref_location_data,  1)
print(ref_location_data.shape)
Sustainability Auditorium | SUS 81-1130
#find the distance from colsest frame and its index
a = np.matmul(com_location_data, ref_location_data)
dist = np.sqrt(np.sum(np.square(np.matmul(com_location_data, ref_location_data)),0))
print(dist.shape)
correspondence_dist = np.min(dist, 1)
correspondence_index = np.argmin(dist,1)
# print(correspondence_dist)
# print(correspondence_index)
# print(com_frame_names)
dataframe_array = np.stack((com_frame_names, correspondence_index, correspondence_dist),axis=1)

df = pd.DataFrame(dataframe_array, columns = ['frame','associated_frame','distance'])
df['associated_frame'] = df['associated_frame'].map(ref_frame_names_dict)
print(df.shape)
#control the type of frame names
#df['associated_frame'] = df['associated_frame'].astype(int)
#df['frame'] = df['frame'].astype(int)
#print(df.shape)


df.to_csv(output_file, header=None, index=None, sep=',', mode='w')

'''
print(day1_x.shape)
print(day1_x)


print(day2_x)

print(day2_x.shape)
print(day2_x)

dist = np.matmul(day1_x, day2_x)
'''                                                                                                                                                                                         
