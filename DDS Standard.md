###### tags: `Robotics`

# DDS Standard

> 2019-10-15

Explain how dds discovery mechanism works.

## DDSI-RTPS

DDS Interoperability Wire Protocol[^1] ensures that applications based on **different vendors** implementations of DDS can **interoperate**.

[^1]: https://www.omg.org/spec/DDSI-RTPS/About-DDSI-RTPS/

RTPS protocal is described in PIM (Platform Independent Model) and PSM (Platform Specfic Model).
PIM: UML class diagrams

:::info
- [x]  The Structure Module
- [ ]  The Messages Module
- [ ]  The Behavior Module
- [ ]  The Discovery Module
:::

### The Structure Module

Entities in black is defined in DDSI-RTPS, and blue is defined in DDS.[^2] There is an one-to-one correspondence between RTPS enties and DDS entites.
<!--->![](https://i.imgur.com/v4tikMZ.png)![](https://i.imgur.com/E9SMCi2.png)<--->
![](https://i.imgur.com/huHknBc.png)

![](https://i.imgur.com/27huPHj.png)



[^2]: https://www.omg.org/spec/DDS/





#### GUID_t

Identify Entites.

![](https://i.imgur.com/Q9wejch.png)


[^3]: https://fast-rtps.docs.eprosima.com/en/latest/introduction.html

#### HistoryCache
- part of the interface between DDS and RTPS

![](https://i.imgur.com/ZK77y6S.png)


> The unit of communication is called a <ins>**Change**</ins>, which represents an update to a topic.[^3]

- On the <ins>writer side</ins>, the HistoryCache contains the *partial* history of changes to data-objects made by the corresponding DDS Writer that are needed to service existing and future matched RTPS Reader endpoints.

- On the <ins>reader side</ins>, it contains the partial superposition of changes to data-objects made by all the matched RTPS Writer endpoints.

> Things to note:
> 1. the word **partial** depends on the QoS of the related DDS entites.
> 2. How a DDS Entity interacts with the HistoryCache is implementation specific.

### Message
Describe messages that are exchanged between the RTPS Writer endpoints and RTPS Reader endpoints.
![](https://i.imgur.com/ZtT9GqQ.png)

3 important submessages:
- **DATA**: This submessage is sent from a Writer to a Reader with information regarding a change to a data-object belonging to a Writers.
- **HEARTBEAT**: This submessage is sent from a Writer to a Reader communicating the CacheChanges that the Writer has available at this moment. (plays a key role in the reliability model)
- **ACKNACK**: This submessage is sent from a Reader to a Writer, and allows the Reader to notify the Writer about which changes it has received and which ones are still missing. It can be used to do both positive and negative acknowledgements.

### Discovery

Implementations must support at least **Simple Participant Discovery Protocol (SPDP)** and **Simple Endpoint Discovery Protocol (SEDP)**. RTPS allows the use of different protocals.
> SPDP + SEDP = SDP (Simple Discovery Protocol)
:::warning
the role of a discovery protocol is to provide information on discovered remote ***Endpoints***. How this information is used by a ***Participant*** to configure its local ***Endpoints*** depends on the actual implementation.

Endpoints allows the implementation to configure:
- The RTPS ***ReaderLocator*** objects that are associated with each RTPS ***StatelessWriter***.
- The RTPS ***ReaderProxy*** objects associated with each RTPS ***StatefulWriter***.
- The RTPS ***WriterProxy*** objects associated with each RTPS ***StatefulReader***
:::

For each DomainParticipant (Participant in RTPS), there are six objects automatically created for discovery purposes:
![](https://i.imgur.com/r48WSKf.png)

Each built-in Endpoint has a EntityID

```c
#define ENTITYID_RTPSParticipant  0x000001c1
#define ENTITYID_SEDP_BUILTIN_TOPIC_WRITER 0x000002c2
#define ENTITYID_SEDP_BUILTIN_TOPIC_READER 0x000002c7
#define ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER  0x000003c2
#define ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER  0x000003c7
#define ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER 0x000004c2
#define ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER  0x000004c7
#define ENTITYID_SPDP_BUILTIN_RTPSParticipant_WRITER  0x000100c2
#define ENTITYID_SPDP_BUILTIN_RTPSParticipant_READER  0x000100c7
...
```

#### SPDP

This phase uses best-effort communications.

Detect the presence of Participants in a domain.
For each Participant, the SPDP creates two RTPS built-in Endpoints[^4]:
[^4]: DDSI-RTPS  §8.5.3

Participant Built-in DataWriter and DataReader (Best-Effort)
- **SPDPbuiltinParticipantWriter**
- **SPDPbuiltinParticipantReader**.

The packet contain the participant’s GUID, its **locators**, and some of the QoS of the participant.

HistoryCache of the SPDPbuiltinParticipantReader will contain information on all active discovered participants.

<ins>multicast address:</ins>

![](https://i.imgur.com/UrZGwkf.jpg)

PDP Packet:

![](https://i.imgur.com/tu5zMTL.png)




#### SEDP

This phase uses reliable communications.

DATA message contains the types, topic names, and QoS policies

| discovery of DataWriters | discovery of DataReaders |
| -------- | -------- |
| SEDPbuiltinPublicationsWriter SEDPbuiltinPublicationsReader | SEDPbuiltinSubscriptionsWriter SEDPbuiltinSubscriptionsReader  |

![](https://i.imgur.com/MU47ZOu.png)


---

#### determine port number
User-defined Endpoints

![](https://i.imgur.com/BpQYhwQ.png)

Built-in Endpoints
![](https://i.imgur.com/1Ngp7h9.png)


<!---
### Synchronization in DDS
1. #### Listener
2. #### WaitSet
--->

## References

- https://docplayer.net/20078263-Introduction-to-dds-www-rti-com-gerardo-pardo-castellote-ph-d-co-chair-omg-dds-sig-cto-real-time-innovations-gerardo-pardo-rti.html