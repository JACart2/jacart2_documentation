# Cart Initialization

## From A Fresh Laptop

1. Install [Ubuntu 22.04.5](https://releases.ubuntu.com/jammy/)
2. Install git: `apt install git`
3. Clone the docker_files project repository:
   `git clone https://github.com/JACart2/docker_files`
4. Run the `laptop-setup.sh` bash script to automatically initialize the host laptop.
5. Run the `run.sh` bash script to initialize the dockerfiles. On first build, this will take a while. Later initializations will cache all unchanged parts of the docker.

## Possible problems
1. The Velodyne LiDAR typically uses a default IP address of 192.168.1.201, so you'll need to configure your laptop to be on the same subnet but with a different IP address, such as 192.168.1.100 in order to receive data. 
2. The control laptop needs to be running NVidia drivers so that it can interface with CUDA.