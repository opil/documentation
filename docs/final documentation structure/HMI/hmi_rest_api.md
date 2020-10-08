## UUID
#### Get a new UUID/v4
>    [GET] http://serveraddress/api/uuid

## Floorplans
#### Get all floorplans
>    [GET] http://serveraddress/api/fp

Response is a json list of objects like below.
#### Get a floorplan
>    [GET] http://serveraddress/api/fp/{id}

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
>    [POST] http://serveraddress/api/fp

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
Floorplan image is saved in: /public/uploads/ and is retrievable from http://serveraddress/{imgurl}.
#### Update a floorplan
>    [PUT] http://serveraddress/api/fp/{id}

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
>    [DELETE] http://serveraddress/api/fp/{id}

Parameter:
id - Mongo _id of the floorplan document 

Response is the deleted floorplan object in json.

***

## Users

#### Get all users
>    [GET] http://serveraddress/api/user

#### Get a user
>    [GET] http://serveraddress/api/user/{id}

#### Create a new user
>    [POST] http://serveraddress/api/user

#### Update a user
>    [PUT] http://serveraddress/api/user/{id}

#### Delete a user
>    [DELETE] http://serveraddress/api/user/{id}

***

## Configs

#### Get all configs
>    [GET] http://serveraddress/api/user

#### Get a config
>    [GET] http://serveraddress/api/user/{id}

#### Create a new config
>    [POST] http://serveraddress/api/user

#### Update a config
>    [PUT] http://serveraddress/api/user/{id}

#### Delete a config
>    [DELETE] http://serveraddress/api/user/{id}

***

## HMIButtons
#### Get all HMIButtons
Request:
>    [GET] localhost/api/hmibutton

Response:
```json
[
    {
        "_id": "5d600ad3283a7d43b8d06abb",
        "ocb_id": "123245489",
        "text": "Call AGV",
        "user_id": "user1",
        "created": "2019-08-23T15:48:35.750Z",
        "updated": "2019-08-23T15:48:35.750Z",
        "__v": 0
    }
]
```
#### Get a HMIButton
Request:
>    [GET] localhost/api/hmibutton/5d600ad3283a7d43b8d06abb

Response:
```json
{
    "_id": "5d600ad3283a7d43b8d06abb",
    "ocb_id": "123245489",
    "text": "Call AGV",
    "user_id": "user1",
    "created": "2019-08-23T15:48:35.750Z",
    "updated": "2019-08-23T15:48:35.750Z",
    "__v": 0
}
```
#### Create a HMIButton
Request: (application/json OR application/x-www-form-urlencoded)
>    [POST] localhost/api/hmibutton
```json
{
    "ocb_id": "123245489",
    "text": "Call AGV",
    "user_id": "user1"
}
```
Response:
```json
{
    "_id": "5d600ad3283a7d43b8d06abb",
    "ocb_id": "123245489",
    "text": "Call AGV",
    "user_id": "user1",
    "created": "2019-08-23T15:48:35.750Z",
    "updated": "2019-08-23T15:48:35.750Z",
    "__v": 0
}
```
#### Update a HMIButton
Request: (application/json OR application/x-www-form-urlencoded)
>    [PUT] localhost/api/hmibutton/5d600ad3283a7d43b8d06abbCaa
```json
{
    "ocb_id": "123245489",
    "text": "AGV call",
    "user_id": "user1"
}
```
Response:
```json
{
    "_id": "5d600ad3283a7d43b8d06abb",
    "ocb_id": "123245489",
    "text": "AGV call",
    "user_id": "user1",
    "created": "2019-08-23T15:48:35.750Z",
    "updated": "2019-08-23T15:52:19.331Z",
    "__v": 0
}
```
#### Delete a HMIButton
Request:
>    [DELETE] localhost/api/hmibutton/5d5d6eea2764b9dd289e6266

Response:
```json
{
    "_id": "5d600ad3283a7d43b8d06abb",
    "ocb_id": "123245489",
    "text": "AGV call",
    "user_id": "user1",
    "created": "2019-08-23T15:48:35.750Z",
    "updated": "2019-08-23T15:52:19.331Z",
    "__v": 0
}
```

#### Create a Subscription to HMI local db
Request: (application/json OR application/x-www-form-urlencoded)
>    [POST] localhost/api/subscription

Parameter subs_id should be the subscription id got when a subscription is created through NGSI Proxy
```json
{
    "subs_id": "123245489"
}
```
Response:
```json
{
    "_id": "5d600ad3283a7d43b8d06abb",
    "subs_id": "123245489",
    "created": "2019-08-23T15:48:35.750Z",
    "updated": "2019-08-23T15:48:35.750Z",
    "__v": 0
}
``` 
