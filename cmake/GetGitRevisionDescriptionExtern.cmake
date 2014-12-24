# this file is executed outside of ApproxMVBB to get the revision describtion
include(cmake/GetGitRevisionDescription.cmake)
git_describe(ApproxMVBB_VERSION "--tags" "--abbrev=0")
message(STATUS "${ApproxMVBB_VERSION}")