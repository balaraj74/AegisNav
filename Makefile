.PHONY: test demo verify clean-demo

test:
	./tools/run_core_tests.sh

demo:
	./tools/run_demo.sh

verify:
	./tools/verify_repo.sh

clean-demo:
	rm -rf artifacts/demo
