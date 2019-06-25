
# server

## Project setup
```
npm install
```

Run `./modify_lib.sh` ONCE

### Compiles and hot-reloads for development
```
npm run serve
```

### Compiles and minifies for production
```
npm run build
```

After build & install nginx
Please revise file 'web-chicken' and copy it to '/etc/nginx/sites-available/web-chicken'
Create a symbolic link using
```
sudo ln -s /etc/nginx/sites-available/web-chicken /etc/nginx/sites-enabled/web-chicken
```

### Run your tests
```
npm run test
```

### Lints and fixes files
```
npm run lint
```

### Customize configuration
See [Configuration Reference](https://cli.vuejs.org/config/).
