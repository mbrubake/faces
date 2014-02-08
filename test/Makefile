#!/bin/bash

export PATH := ../bin:$(PATH)
n=100

raw: capture.config
	rm -rf raw/
	mkdir -p raw
	capture -o raw/ -n $(n)
seg: raw raw/0.pcd behead.config ../bin/behead
	rm -rf seg/
	mkdir -p seg
	behead -i raw/ -o seg/ -n $(n)
stitch: seg seg/0.pcd stitch.config ../bin/stitch
	rm -rf stitch/
	mkdir -p stitch
	stitch -i seg/ -o stitch/
global_stitch: seg seg/0.pcd stitch.config ../bin/global_stitch
	rm -rf global_stitch/
	mkdir -p global_stitch
	global_stitch -i seg/ -o global_stitch/
mesh.stl: stitch stitch/0.pcd graft.config
	graft -i stitch/merged.pcd -o ./
	view.py mesh.stl
clean:
	rm -r raw
	rm -r seg
	rm -r stitch
	rm mesh.stl
