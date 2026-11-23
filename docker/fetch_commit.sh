#!/usr/bin/env bash
# fetch_commit.sh <git-url> <commit-sha> <dir>
# Shallow-fetch exactly one pinned commit (GitHub serves any reachable SHA), so image builds
# are reproducible without cloning full histories.
set -euo pipefail
url=$1
sha=$2
dir=$3
mkdir -p "$dir"
cd "$dir"
git init -q
git remote add origin "$url"
git fetch -q --depth 1 origin "$sha"
git checkout -q FETCH_HEAD
echo "$url $sha" > .stride_provenance
