
include(cmake/GetGitRevisionDescription.cmake)
git_describe(ApproxMVBB_VERSION)
message(STATUS "${ApproxMVBB_VERSION}")