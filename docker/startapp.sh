#!/bin/sh
# Launched by the base image inside its X session. One job: the Console.
cd /app
exec python3 console/app.py
