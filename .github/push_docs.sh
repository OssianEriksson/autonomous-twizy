#!/bin/bash

# Fail if any sub-command fails
set -e

cd ~/autonomous-twizy/docs

# Commit using the github-actions user
git config --global user.email "github-actions[bot]@users.noreply.github.com"
git config --global user.name "github-actions[bot]"

# Stage everything
git add -A

# Commit and push
commit_hash=`git -C ~/autonomous-twizy/src log -n 1 --pretty=format:"%H"`
git commit -m "Docs for $commit_hash"
git push "https://$GITHUB_ACTOR:$GITHUB_TOKEN@github.com/$GITHUB_REPOSITORY.git" --force
