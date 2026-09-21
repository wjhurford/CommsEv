# Fleets built in the Console are saved here.
#
# This folder is deliberately empty. A fleet is a composition - which agents,
# where, on whose network, under which doctrine - and every one of those is a
# decision. Decisions are made in the Console, not typed into a file here.
#
# To make one:  Setup tab -> Blue fleet -> "+ Build custom fleet..."
# Pick agent types from agents/, add rows, name it, save. It appears in this
# folder and in the dropdown from then on.
#
# The hardware a fleet is composed from lives in agents/ and is read-only in
# spirit: an agent file describes a purchasable item. See agents/roboracer.yaml.
#
# The fixtures the test suite needs live in tests/fixtures/fleets/ so they
# never appear in the dropdown alongside fleets built in the Console.
