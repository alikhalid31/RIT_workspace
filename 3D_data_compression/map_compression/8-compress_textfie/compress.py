import zlib, base64, os
import sys

# python [path to dataset] [input_folder] [output_folder (compress data)] [recover_data_folder]
#S = base64.b64encode(zlib.compress(T))
#T = zlib.decompress(base64.b64decode(S))

data_path = sys.argv[1]
input_folder = data_path + '/' + sys.argv[2] 
output_folder = data_path + '/' + sys.argv[3]
recover_folder = data_path + '/' + sys.argv[4]

#print(input_folder)
#print(output_folder)
#print(recover_folder)
if not os.path.exists(output_folder):
    print("here")
    os.makedirs(output_folder)

if not os.path.exists(recover_folder):
    os.makedirs(recover_folder)

for filename in os.listdir(input_folder):
    input_file = os.path.join(input_folder, filename)
    output_file = os.path.join(output_folder, os.path.splitext(os.path.basename(filename))[0]+".zlib")
    recover_file = os.path.join(recover_folder, filename)

    with open(input_file, "rb") as myfile:
        data = myfile.read()

    compressed_data = zlib.compress(data)
    with open(output_file, "wb") as myfile:
        myfile.write(compressed_data)
    
    recover_data = zlib.decompress(compressed_data)
    with open(recover_file, "wb") as myfile:
        myfile.write(recover_data)



