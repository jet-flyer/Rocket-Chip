docs:
	@echo "Generating state diagram..."
	# TODO: Extract states to JSON
	python tools/state_to_dot.py states.json > states.dot
	dot -Tsvg states.dot -o Docs/states.svg
