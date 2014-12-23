
include(cmake/GetGitRevisionDescription.cmake)
git_describe(ApproxMVBB_VERSION "--tags")
message(STATUS "${ApproxMVBB_VERSION}")