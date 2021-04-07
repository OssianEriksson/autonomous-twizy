#!/bin/bash

# Get SHAs of commits which were merged for pull request
parents=`git -C "$GITHUB_WORKSPACE/src" log -1 --merges --pretty=format:%P`

# If no commits were merged (ie test was ran on standalone commit) use the hash of that commit
parents=${parents:-$GITHUB_SHA}

# Call Github's statuses API
curl --request POST \
--url "$GITHUB_API_URL/repos/$GITHUB_REPOSITORY/statuses/${parents#* }" \
--header "Authorization: Bearer $GITHUB_TOKEN" \
--header "Content-Type: application/json" \
--header "Accept: application/vnd.github.v3+json" \
--data '{
    "state": "'"$1"'",
    "context": "'"$2"'",
    "description": "'"$3"'"
}'