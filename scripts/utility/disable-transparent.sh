#!/bin/bash

echo $1 > /sys/kernel/mm/transparent_hugepage/enabled
