# Setting up local computer and vehicle remote connections
Other than sharing ROS master, the other steps here are optional but help in speeding up the setup process before test runs. Getting a passwordless sudo is necessary to get the emergency remote kill switch to work. Without a passwordless sudo, the jetson_gpio package will not be able to be run by the ROS server.

## Sharing ROS master
The car and a local computer have to share a common ROS master to run the tests. To do this, they must first be connected to the same network. Then, check the assigned IP on both computers by:
```bash
$ hostname -I
```
Assuming the local computer IP is 10.42.0.1 and the Jetson 10.42.0.2, on the local computer, in every terminal before running the ROS commands run these first
```bash
$ export ROS_MASTER_URI=http://10.42.0.2:11311
$ export ROS_IP=10.42.0.1
```
You can put these lines inside ~/.bashrc to set the ROS master by default. To run your own master, simply reverse the changes:
```bash 
$ export ROS_MASTER_URI=http://localhost:11311
$ unset ROS_IP
```

## Getting passwordless sudo
In the jetson, run the command
```bash
$ sudo visudo
```
On the last line, add the following:
```
username ALL = (ALL) NOPASSWD: ALL
```
replacing username with your user name.  
Note: it is important to use visudo to edit the file instead of manually editing it with other editors as an invalid file will lead to loss of access to sudo privileges.

## Synchronizing time across machines
If the system time in the different machines are not synchronized, the timestamp on ROS messages will differ and this may cause problems. To synchronize the system time, install Chrony on both computers:
```bash
$ sudo apt-get install ntpdate chrony
```
Edit the configuration file `/etc/chrony/chrony.conf` and add the following:  
On the local computer,
```
local stratum 8
allow jetson_ip
```
replacing car_ip with the IP address of the jetson.  
On the Jetson,
```
server local_ip iburst
```
replacing local_ip with the IP address of the local computer.

## Setting up static IP addresses
If a Wi-Fi adapter or router is used specifically for the connection between the local computer and remote RC car, static IP addresses can be assigned to both computers.

Using a Wi-Fi adapter (suppose the static IP to be assigned to the car is 10.42.0.2):
1. Connect the USB adapter to the local computer
2. Create a new Wi-Fi network , choosing the corresponding Wi-Fi adapter.
3. On the IPv4 settings, select the dropdown for Method to be Shared to other computers
4. On the Jetson, connect to the created Wi-Fi network.
5. Edit the network connection such that:  
   In the general tab both 'Automatically connect to this network when it is available' and 'All users may connect to this network' are checked  
   In the IPv4 Settings tab the Method is set to Manual and add an address with  
   Address: 10.42.0.2  
   Netmask: 24  
   Gateway: 10.42.0.1 (IP of host)  
   DNS Servers: 8.8.8.8, 8.8.4.4
6. Disconnect and connect to the network again.

## Setup SSH shortcut
To avoid having to type in `ssh username@hostname` or `ssh username@hostip` everytime to access the remote terminal, one can setup an SSH config file to to create a shortcut for SSH to the host. (Instructions obtained from [here](http://nerderati.com/2011/03/17/simplify-your-life-with-an-ssh-config-file/)).
```bash
$ nano ~/.ssh/config
```
Enter the following:
```
Host shortcutname
HostName hostname
Port portnumber
User username
```
with the corresponding shortcut name, host name, port number and username. The default port number is 22.  
Then,
```bash
$ cd /etc/bash_completion.d/
$ sudo nano ssh
```
Copy and paste the following:
```
_ssh() 
{
    local cur prev opts
    COMPREPLY=()
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"
    opts=$(grep '^Host' ~/.ssh/config | grep -v '[?*]' | cut -d ' ' -f 2-)

    COMPREPLY=( $(compgen -W "$opts" -- ${cur}) )
    return 0
}
complete -F _ssh ssh
```
From here onwards, the command `ssh shortcutname` will be equivalent to entering `ssh username@hostname -p portnumber`.

## Setting RSA key
Another inconvenience when accessing remote terminal is the need to enter the password everytime. However, by setting up the RSA key one can eliminate the need to do so. While connected to the same network, on the local computer:
```bash
$ ssh-keygen
  press enter or type your preferred file name for storing the key  
  press enter or type a password (this is a password to bet set on the stored key, not the user account password)  
  press enter again or type the same password  
$ chmod 755 ~/.ssh
$ ssh-copy-id -i id_rsa user@hostname (or change id_rsa to the filename you entered)
$ ssh -oHostKeyAlgorithms='ssh-rsa' user@hostname (if it fails, change ssh-rsa to the type shown in ~/.ssh/known_hosts)
$ chmod 600 ~/.ssh/authorized_keys
$ exit exit
$ ssh-agent bash
$ ssh-add ~/.ssh/id_rsa (or change id_rsa to the filename you entered)
$ exit
```
This needs to be done only the first time you use SSH to access the Jetson terminal. After this you can simply use the `ssh user@hostname` command
