#!/usr/bin/env python3
# -*-coding:utf8-*-
# Install piper_sdk before running this demo.
from piper_sdk import C_PiperInterface_V2


if __name__ == "__main__":
    piper = C_PiperInterface_V2()
    piper.ConnectPort()
    nodes = (1, 2, 3, 4, 5, 6, 7, 15)
    versions = piper.GetMotorDriverVersions(nodes=nodes)
    for node in nodes:
        print("node", node, ":", piper.GetNodeFirmwareInfo(node))
    print(versions)
