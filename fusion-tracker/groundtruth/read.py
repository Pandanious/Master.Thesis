#! /usr/bin/env python3

gt_path = './walk_straight_forwardlook.txt'
gt = open(gt_path,'r')

st = gt.readline()
result = st[st.find('(')+1:st.find(')')]
li = list(map(int, result.split(',')))
print(li)