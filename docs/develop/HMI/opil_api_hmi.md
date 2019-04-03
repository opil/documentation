## REST API

### Floorplans

#### Get all floorplans
>    [GET] http://serveraddress:8081/api/fp

Response is a json list of objects like below.

#### Get a floorplan
>    [GET] http://serveraddress:8081/api/fp/{id}

Parameter:
id - Mongo _id of the floorplan document

Example response (json):
```json
{   "_id":"5c481eb5d543750018c7c2a9",
    "fieldname":"floorplan",
    "originalname":"fp.png",
    "encoding":"7bit",
    "mimetype":"image/png",
    "size":144208,
    "filename":"floorplan-1548230325552.png",
    "imgurl":"uploads/floorplan-1548230325552.png",
    "name":"FP",
    "scale":50,
    "xoffset":5,
    "yoffset":5,
    "created":"2019-01-23T07:58:45.561Z",
    "updated":"2019-01-23T07:58:45.566Z","__v":0
}
```

#### Create a floorplan
>    [POST] http://serveraddress:8081/api/fp

Use enctype: 'multipart/form-data'

Parameters example (html):

```html
<input type="file" name="floorplan" required />
<input type="text" name="name" required />
<input type="number" name="scale" required />
<input type="number" name="xoffset" required />
<input type="number" name="yoffset" required />
```

Example response (json):
```json
{   "_id":"5c481eb5d543750018c7c2a9",
    "fieldname":"floorplan",
    "originalname":"fp.png",
    "encoding":"7bit",
    "mimetype":"image/png",
    "size":144208,
    "filename":"floorplan-1548230325552.png",
    "imgurl":"uploads/floorplan-1548230325552.png",
    "name":"FP",
    "scale":50,
    "xoffset":5,
    "yoffset":5,
    "created":"2019-01-23T07:58:45.561Z",
    "updated":"2019-01-23T07:58:45.566Z","__v":0
}
```

Floorplan image is saved in: /public/uploads/ and is retrievable from http://serveraddress:8081/{imgurl}.

#### Update a floorplan
>    [PUT] http://serveraddress:8081/api/fp/{id}

Parameter:
id - Mongo _id of the floorplan document

Update floorplan with only fields: `name`, `scale`, `xoffset`, `yoffset`.

Image is not possible to update, you should delete and create a new floorplan.

Use either of enctypes for the payload:
- application/x-www-form-urlencoded
- application/json
```json
    {
        "name": "newname",
        "scale": 40,
        "xoffset": 4,
        "yoffset": 4
    }
```

Response is the updated floorplan object in json.

#### Delete a floorplan
>    [DELETE] http://serveraddress:8081/api/fp/{id}

Parameter:
id - Mongo _id of the floorplan document

Response is the deleted floorplan object in json.

***

### Users

#### Get all users
>    [GET] http://serveraddress:8081/api/user

#### Get a user
>    [GET] http://serveraddress:8081/api/user/{id}

#### Create a new user
>    [POST] http://serveraddress:8081/api/user

#### Update a user
>    [PUT] http://serveraddress:8081/api/user/{id}

#### Delete a user
>    [DELETE] http://serveraddress:8081/api/user/{id}
