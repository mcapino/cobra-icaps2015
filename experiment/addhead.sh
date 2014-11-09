#!/bin/bash
cat "instances/$1/head" > "instances/$1/data.out.head"
cat "instances/$1/data.out" >> "instances/$1/data.out.head"
