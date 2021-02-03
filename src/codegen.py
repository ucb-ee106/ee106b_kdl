import sys
codefile = open(sys.argv[1])

with open(sys.argv[1], 'r') as f:
	for line in f:
		if not line:
			continue
		line = line.replace('][', ', ')
		line = line.replace('[', '(')
		line = line.replace(']', ')')
		print(line.strip())
