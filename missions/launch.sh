#!/bin/bash

# Menambahkan direktori bin/ proyek ini ke dalam PATH
export PATH=$PATH:$(pwd)/../bin

echo "Menjalankan witmotion.moos..."
pAntler witmotion.moos
