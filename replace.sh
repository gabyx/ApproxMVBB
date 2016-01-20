#!/bin/bash
shopt -s expand_aliases
source ~/.bash_aliases
fileRegexReplace -f -r 's@ApproxMVBB_Exception_INCLUDE_FILE@ApproxMVBB_Exception_INCLUDE_FILE@g' -R ".*[hpp|cpp]" -e "*external*" -e "*.git*" ./
fileRegexReplace -f -r 's@ApproxMVBB_AssertionDebug_INCLUDE_FILE@ApproxMVBB_AssertionDebug_INCLUDE_FILE@g' -R ".*[hpp|cpp]" -e "*external*" -e "*.git*" ./
fileRegexReplace -f -r 's@ApproxMVBB_MyMatrixTypeDefs_INCLUDE_FILE@ApproxMVBB_MyMatrixTypeDefs_INCLUDE_FILE@g' -R ".*[hpp|cpp]" -e "*external*" -e "*.git*" ./
fileRegexReplace -f -r 's@ApproxMVBB_Platform_INCLUDE_FILE@ApproxMVBB_Platform_INCLUDE_FILE@g' -R ".*[hpp|cpp]" -e "*external*" -e "*.git*" ./
fileRegexReplace -f -r 's@ApproxMVBB_StaticAssert_INCLUDE_FILE@ApproxMVBB_StaticAssert_INCLUDE_FILE@g' -R ".*[hpp|cpp]" -e "*external*" -e "*.git*" ./
fileRegexReplace -f -r 's@ApproxMVBB_AABB_INCLUDE_FILE@ApproxMVBB_AABB_INCLUDE_FILE@g' -R ".*[hpp|cpp]" -e "*external*" -e "*.git*" ./
fileRegexReplace -f -r 's@ApproxMVBB_OOBB_INCLUDE_FILE@ApproxMVBB_OOBB_INCLUDE_FILE@g' -R ".*[hpp|cpp]" -e "*external*" -e "*.git*" ./