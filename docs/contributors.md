# Contributors

### Checkin

- Do check in source (`src/`)
- Do check in a single `index.js` file after running `ncc`
- Do not check in `node_modules/`

### NCC

In order to avoid uploading `node_modules/` to the repository, we use [zeit/ncc](https://github.com/zeit/ncc) to create a single `index.js` file that gets saved in `dist/`.

### Developing

If you're developing locally, you can run
```
npm install
tsc
ncc build src/setup-catkin.ts
```
Any files generated using `tsc` will be added to `lib/`, however those files also are not uploaded to the repository and are exluded using `.gitignore`.

During the commit step, Husky will take care of formatting all files with [Prettier](https://github.com/prettier/prettier)
