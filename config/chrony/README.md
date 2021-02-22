# Chrony

Follow this guide to time synchronize a robot (or multiple robots) with a ground station using chrony.
The time synchronization follows a master-slave architecture in which each robot connects to the master.

## Install server

The chrony server should be installed on the ground station.
Make sure the `allow` option is set to the local network.

```bash
sudo apt-get install -y chrony
sudo cp chrony_server.conf /etc/chrony/chrony.conf
sudo service chrony restart
```

## Install client

The chrony client should be installed on the robot.
Make sure the `server` option points to the IP address of the ground station.

```bash
sudo apt-get install -y chrony
sudo cp chrony_client.conf /etc/chrony/chrony.conf
sudo service chrony restart
```

## Test chrony

Verify that the server gets time updates from NTP servers and that the client lists the ground station as the only source.

```bash
chronyc sources
watch chronyc tracking
```
