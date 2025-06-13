import numpy as np

# Nombres de archivo
input_file = '../ouster/go/gnss_go.txt'

with open(input_file, 'r') as f:
    header = f.readline().strip().split()
    data = np.loadtxt(f)

idx_lat = header.index("latitude")
idx_lon = header.index("longitude")

data[:, [idx_lat, idx_lon]] = data[:, [idx_lon, idx_lat]]

header[idx_lat], header[idx_lon] = header[idx_lon], header[idx_lat]

with open(input_file, 'w') as f:
    f.write('\t'.join(header) + '\n')
    np.savetxt(f, data, fmt='%.8f', delimiter='\t')


