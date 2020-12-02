#!/bin/sh
curl --location --request POST 'http://130.188.160.100:1026/v2/subscriptions/' --header 'Content-Type: application/json' --data-raw '{
  "description": "Notify QL Proxy Server about changes in SensorAgent",
  "subject": {
    "entities": [
      {
        "idPattern": ".*",
        "type": "SensorAgent"
      }
    ],
    "condition": {
      "attrs": []
    }
  },
  "notification": {
    "http": {
      "url": "http://130.188.160.100:4440"
    },
    "attrs": [],
    "metadata": ["dateCreated", "dateModified"]
  }
}'
