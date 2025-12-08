#!/bin/bash

echo "Setting up paths for Dahua camera app building and packaging..."

if [ -d "/opt/arm-himix200-linux-v2/bin" ]; then
    export PATH="/opt/arm-himix200-linux-v2/bin:$PATH"
    echo "Compiler added to PATH"
else
    echo "Warning: /opt/arm-himix200-linux-v2/bin not found"
fi

if [ -d "/opt/dhop_sdk/Include/DhopSdk" ]; then
    export PATH="/opt/dhop_sdk/Include/DhopSdk:$PATH"
    echo "Dhop libs added to PATH"
else
    echo "Warning: /opt/dhop_sdk/Include/DhopSdk not found"
fi

if [ -d "/opt/dhop_sdk/Tools" ]; then
    export PATH="/opt/dhop_sdk/Tools:$PATH"
    echo "ezyBuilder added to PATH"
else
    echo "Warning: /opt/dhop_sdk/Tools not found"
fi

