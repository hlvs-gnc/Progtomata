# Define JSON files to be cleaned
TIL_JSON := til.json
LI_JSON := li.json

# Define the source directories for the insert command
SRC_DIR := src/
LIB_DIRS := $(filter-out lib/trice/, $(wildcard lib/*/))  # Get all directories inside lib/ except lib/trice

.PHONY: clean update

# Clean target: empties til.json and li.json while keeping the files
trice_clean:
	@echo "Cleaning JSON files..."
	@> $(TIL_JSON)
	@> $(LI_JSON)
	@echo "Files cleaned: $(TIL_JSON) and $(LI_JSON)"

# Update target: runs trice insert with specified options
trice_update:
	@echo "Updating project source with trice insert..."
	trice insert -src $(SRC_DIR) $(foreach dir, $(LIB_DIRS), -src $(dir))
	@echo "Update complete."
