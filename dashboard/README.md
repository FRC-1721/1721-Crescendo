# FRC 1721 Dashboard
## Getting Started
- Make sure you have sed installed
- Make sure Robot sim is running
- Make sure npm is installed

Make sure you are in the dashboard/ folder.

Install deps

``` sh
npm ci
```

Build webpage

``` sh
make build
```

(NOT REQUIRED) You can build automatically when a file changes in src/ (requires 'entr', `paru -S entr` `paman install entr`)

``` sh
make dev
```

Run webpage

``` sh
make run
```

`make run` can be left running when you run `make build`, and doesn't need to be run again to update the webpage
