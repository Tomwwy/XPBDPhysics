
- replace std hashset to set in unordered_dense
- reduce BVH building, not rebuilding every sub step, e.g. use insert/update, or delay if no big change (allow slight error)