#!/bin/bash
currentDate=$(date +'%d/%m/%Y'); echo "export const releaseDate = \"$currentDate\";" > src/releaseDate.ts